// SoftwareSerial_nRF52_Freertos.cpp
// Blocking API w/ cooperative yielding (Option 1)

#include <Arduino.h>
#include <softwareserial/SoftwareSerial.hpp>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_timer.h>

#include "FreeRTOS.h"
#include "task.h"

// ======= Timing / Limits =======
#define MAX_BAUDRATE            1000000UL          // practical max for this sw impl
#define TIMER_PRESCALER         4                  // 1 MHz timer (1 us tick) @ 64 MHz CPU
#define TX_WAIT_TIMEOUT_MS      50                 // safety timeout for TX wait loops
#define RX_WAIT_PAUSE_MS        1                  // small delay when polling reads

// Optional extra latency correction for the first RX sample (~1-2 us typical)
#define ATTACHINT_LATENCY_US    1U                 // keep integer (used with us math)

// ======= External TX buffer OE (74LVC2G241) =======
#define CONTROL_PIN             10                 // HIGH = drive TX line (enable buffer)

static inline void setupOEPins() {
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW); // start in RX (TX buffer disabled)
}
static inline void enableTXBuffer() { digitalWrite(CONTROL_PIN, HIGH); }
static inline void enableRXBuffer() { digitalWrite(CONTROL_PIN, LOW); }

// ======= RX/TX shared state =======
#define _SS_MAX_RX_BUFF 64
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

SoftwareSerial *SoftwareSerial::active_object = nullptr;

// RX/TX state flags
static volatile bool _rxInProgress = false;
static volatile bool _txInProgress = false;

// RX state
static volatile uint8_t _rxByte = 0;
static volatile uint8_t _rxBitCount = 0;

// TX state
static volatile uint8_t _txByte = 0;
static volatile uint8_t _txBitCount = 0;

// Timing/cache
static uint32_t _baudRate = 57600;
static uint32_t _bitTimeUs = 17; // ≈ 1e6 / 57600

// Cached GPIO addresses/masks for speed
static uint32_t _rxMask;
static NRF_GPIO_Type *_rxPort;

static uint32_t _txMask;
static uint32_t _txInvMask;
static NRF_GPIO_Type *_txPort;

// Also keep Arduino-ish helpers in case they're used elsewhere
static volatile uint32_t *_transmitPortRegister;
static const volatile uint32_t *_receivePortRegister;
static uint32_t _transmitBitMask;
static uint32_t _receiveBitMask;

// The RX pin we attached to
uint8_t gReceivePin = 0;

// ======= Forward ISR declarations (called by vector wrappers) =======
extern "C" void SoftwareSerial_GPIOTE_IRQHandler(void);
extern "C" void SoftwareSerial_TIMER2_IRQHandler(void);

// ======= Vector binding for TIMER2 only =======
// We keep GPIOTE under Arduino's attachInterrupt() dispatcher.
extern "C" void TIMER2_IRQHandler(void) {
  SoftwareSerial_TIMER2_IRQHandler();
}

// ======= Ctors / Dtor =======
SoftwareSerial::SoftwareSerial() {}

SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /*= false*/) {
  init(receivePin, transmitPin, inverse_logic);
}

SoftwareSerial::~SoftwareSerial() {
  stopListening();
}

// ======= Core init =======
void SoftwareSerial::init(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic) {
  _receivePin   = receivePin;
  _transmitPin  = transmitPin;
  _inverse_logic = inverse_logic;
  gReceivePin   = receivePin;     // IMPORTANT: match the actual RX pin
}

// ======= Begin / hardware setup =======
void SoftwareSerial::begin(long speed) {
  if ((unsigned long)speed > MAX_BAUDRATE) speed = MAX_BAUDRATE;
  _baudRate = (uint32_t)speed;
  _bitTimeUs = 1000000UL / _baudRate;

  // Configure pins
  setTX(_transmitPin);
  setRX(_receivePin);
  setupOEPins();

  // Cache masks/ports
  _rxMask = digitalPinToBitMask(_receivePin);
  _rxPort = digitalPinToPort(_receivePin);

  _txMask = digitalPinToBitMask(_transmitPin);
  _txInvMask = ~_txMask;
  _txPort = digitalPinToPort(_transmitPin);

  // Attach start-bit detector on RX FALLING edge via Arduino
  attachInterrupt(digitalPinToInterrupt(gReceivePin), SoftwareSerial_GPIOTE_IRQHandler, FALLING);

  // Setup TIMER2 for 1 MHz, 32-bit, compare[0] interrupts
  NRF_TIMER2->MODE     = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE  = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER2->PRESCALER= TIMER_PRESCALER;
  NRF_TIMER2->TASKS_STOP   = 1;
  NRF_TIMER2->TASKS_CLEAR  = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

  // Priorities — keep moderate so we don't starve other ISRs
  NVIC_SetPriority(TIMER2_IRQn, 1);
  NVIC_EnableIRQ(TIMER2_IRQn);

  listen();
}

// ======= Listener control =======
bool SoftwareSerial::listen() {
  if (active_object != this) {
    if (active_object) active_object->stopListening();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    return true;
  }
  return false;
}

bool SoftwareSerial::stopListening() {
  if (active_object == this) {
    active_object = nullptr;
    return true;
  }
  return false;
}

// ======= Buffer API =======
int SoftwareSerial::available() {
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

int SoftwareSerial::read() {
  if (_receive_buffer_head == _receive_buffer_tail) return -1;
  uint8_t d = _receive_buffer[_receive_buffer_head];
  _receive_buffer_head = (uint8_t)((_receive_buffer_head + 1) % _SS_MAX_RX_BUFF);
  return d;
}

void SoftwareSerial::readBytes(uint8_t *buffer, size_t length) {
  size_t bytes_read = 0;
  while (bytes_read < length) {
    int c = read();
    if (c >= 0) {
      buffer[bytes_read++] = (uint8_t)c;
    } else {
      // be polite to scheduler on a single core
      vTaskDelay(pdMS_TO_TICKS(RX_WAIT_PAUSE_MS));
    }
  }
}

size_t SoftwareSerial::writeBuffer(const uint8_t *buffer, size_t length) {
  size_t bytes_written = 0;
  for (size_t i = 0; i < length; i++) {
    size_t w = write(buffer[i]);
    if (w == 1) bytes_written++;
    else break; // timeout/error
  }
  return bytes_written;
}

// ======= TX (blocking, but cooperative) =======
size_t SoftwareSerial::write(uint8_t b) {
  // Wait for any prior TX, but yield so other tasks run
  TickType_t start = xTaskGetTickCount();
  while (_txInProgress) {
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(TX_WAIT_TIMEOUT_MS)) {
      // Safety: give up and force-clear the stuck TX
      _txInProgress = false;
      break;
    }
    taskYIELD();
  }

  _txInProgress = true;
  _txByte = b;
  _txBitCount = 0;

  // Disable RX during TX turnaround, drive via buffer OE
  _rxInProgress = false;
  enableTXBuffer();

  // Start bit: line LOW
  *((volatile uint32_t *)&_txPort->OUT) &= _txInvMask;

  // Start/arm TIMER2 fresh
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->CC[0] = _bitTimeUs;
  NRF_TIMER2->TASKS_START = 1;

  // Wait until TX finishes; yield cooperatively
  start = xTaskGetTickCount();
  while (_txInProgress) {
    if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(TX_WAIT_TIMEOUT_MS)) {
      // Timeout: stop timer and bail
      _txInProgress = false;
      NRF_TIMER2->TASKS_STOP = 1;
      // Best-effort line idle and RX re-enable
      _txPort->OUT |= _txMask; // idle HIGH
      enableRXBuffer();
      attachInterrupt(digitalPinToInterrupt(gReceivePin), SoftwareSerial_GPIOTE_IRQHandler, FALLING);
      return 0; // indicate failure
    }
    taskYIELD();
  }

  return 1;
}

// ======= ISR: start-bit detect via attachInterrupt() =======
extern "C" void SoftwareSerial_GPIOTE_IRQHandler() {
  // Temporarily detach to avoid retrigger while we’re sampling this frame
  detachInterrupt(digitalPinToInterrupt(gReceivePin));

  _rxInProgress = true;
  _rxBitCount = 0;
  _rxByte = 0;

  // Prepare and schedule first mid-bit sample
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;

  // Sample at 1.5 bit-times minus a tiny latency correction
  uint32_t first_delay = _bitTimeUs + (_bitTimeUs / 2);
  if (first_delay > ATTACHINT_LATENCY_US) first_delay -= ATTACHINT_LATENCY_US;

  NRF_TIMER2->CC[0] = first_delay;
  NRF_TIMER2->TASKS_START = 1;
}

// ======= ISR: TIMER2 bit timing for RX and TX =======
extern "C" void SoftwareSerial_TIMER2_IRQHandler(void) {
  // Clear compare event immediately
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;

  // ---------- RX state machine ----------
  if (_rxInProgress) {
    // Shift in LSB-first by sampling RX line
    _rxByte >>= 1;
    if ((_rxPort->IN & _rxMask)) {
      _rxByte |= 0x80;
    }
    _rxBitCount++;

    if (_rxBitCount >= 8) {
      // Push to buffer if space
      uint8_t next = (uint8_t)((SoftwareSerial::_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF);
      if (next != SoftwareSerial::_receive_buffer_head) {
        SoftwareSerial::_receive_buffer[SoftwareSerial::_receive_buffer_tail] = _rxByte;
        SoftwareSerial::_receive_buffer_tail = next;
      }

      _rxInProgress = false;

      // Stop timer if TX not active; else TX continues owning it
      if (!_txInProgress) {
        NRF_TIMER2->TASKS_STOP = 1;
        NRF_TIMER2->TASKS_CLEAR = 1;
      }

      // Re-arm start-bit detect
      attachInterrupt(digitalPinToInterrupt(gReceivePin), SoftwareSerial_GPIOTE_IRQHandler, FALLING);
    } else {
      // Schedule next bit sample
      NRF_TIMER2->CC[0] += _bitTimeUs;
    }
  }

  // ---------- TX state machine ----------
  if (_txInProgress) {
    _txBitCount++;

    if (_txBitCount <= 8) {
      // Send data bits LSB-first
      if (_txByte & 0x01) {
        _txPort->OUT |= _txMask;    // HIGH
      } else {
        _txPort->OUT &= _txInvMask; // LOW
      }
      _txByte >>= 1;
      NRF_TIMER2->CC[0] += _bitTimeUs;
    } else if (_txBitCount == 9) {
      // Stop bit (HIGH)
      _txPort->OUT |= _txMask;
      NRF_TIMER2->CC[0] += _bitTimeUs;
    } else {
      // TX complete
      _txInProgress = false;

      // If RX not ongoing, we can stop timer. Otherwise RX keeps it running.
      if (!_rxInProgress) {
        NRF_TIMER2->TASKS_STOP = 1;
      }

      // Re-enable RX buffer immediately
      enableRXBuffer();

      // Re-arm start-bit detect (in case we had disabled it earlier)
      attachInterrupt(digitalPinToInterrupt(gReceivePin), SoftwareSerial_GPIOTE_IRQHandler, FALLING);
    }
  }
}

// ======= Pin helpers =======
void SoftwareSerial::setTX(uint8_t tx) {
  digitalWrite(tx, HIGH); // idle HIGH
  pinMode(tx, OUTPUT);
  NRF_GPIO_Type *port = digitalPinToPort(tx);
  _transmitPortRegister = &(port->OUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  _txMask = _transmitBitMask;
  _txInvMask = ~_txMask;
  _txPort = port;
  _transmitPin = tx;
}

void SoftwareSerial::setRX(uint8_t rx) {
  pinMode(rx, INPUT_PULLUP);
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  NRF_GPIO_Type *port = digitalPinToPort(rx);
  _receivePortRegister = &(port->IN);
  _rxMask = _receiveBitMask;
  _rxPort = port;
}

// ======= Flush RX buffer =======
void SoftwareSerial::flush() {
  if (!isListening()) return;
  // no direct GPIOTE fiddling — just clear buffer
  _receive_buffer_head = _receive_buffer_tail = 0;
}
