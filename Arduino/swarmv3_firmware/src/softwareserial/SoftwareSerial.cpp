#include <Arduino.h>
#include <softwareserial/SoftwareSerial.hpp>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>
#include <nrf_timer.h>

// Assuming 64 MHz CPU clock, 57,600 baud
// Bit time in microseconds:
#define BIT_TIME_US (1000000UL / 57600) // ≈ 17.36 us

// Extra delay to account for attachInterrupt() handler latency
// This will vary slightly between boards, but ~1.5 us works well on nRF52840
#define ATTACHINT_LATENCY_US 1.5

volatile uint8_t rxByte;
volatile bool rxReady = false;

// === CONFIG ===
// Pins — change these as needed
#define RX_PIN 8
#define TX_PIN 6
#define CONTROL_PIN 10 // Output Enable for 74LVC2G241 TX buffer (active HIGH to drive TX line)

// Baudrate max 1M for reliable timing
#define MAX_BAUDRATE 1000000UL

// TIMER config
#define TIMER_PRESCALER 4 // 1 MHz (1 us ticks)

// === BUFFER ===
#define _SS_MAX_RX_BUFF 64
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

SoftwareSerial *SoftwareSerial::active_object = nullptr;

// === STATE ===
static volatile bool _rxInProgress = false;
static volatile bool _txInProgress = false;

static volatile uint8_t _rxByte;
static volatile uint8_t _rxBitCount;

static volatile uint8_t _txByte;
static volatile uint8_t _txBitCount;

static uint32_t _baudRate = 57600;
static uint32_t _bitTimeUs = 17; // default for 57600

static uint32_t _rxMask;
static NRF_GPIO_Type *_rxPort;

static uint32_t _txMask;
static uint32_t _txInvMask;
static NRF_GPIO_Type *_txPort;

static volatile uint32_t *_transmitPortRegister;
static const volatile uint32_t *_receivePortRegister;
static uint32_t _transmitBitMask;
static uint32_t _receiveBitMask;

uint8_t gReceivePin = 0;

void setupOEPins()
{
  pinMode(CONTROL_PIN, OUTPUT);

  // Start with RX enabled, TX disabled
  digitalWrite(CONTROL_PIN, LOW);
}

void enableTXBuffer()
{
  digitalWrite(CONTROL_PIN, HIGH);
}

void enableRXBuffer()
{
  digitalWrite(CONTROL_PIN, LOW);
}

// === ISR DECLARATIONS ===
extern "C" void SoftwareSerial_GPIOTE_IRQHandler(void);
extern "C" void SoftwareSerial_TIMER2_IRQHandler(void);

// === SoftwareSerial methods ===

SoftwareSerial::SoftwareSerial() {}

SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /*= false*/)
{
  _receivePin = receivePin;
  _transmitPin = transmitPin;
  _inverse_logic = inverse_logic;
  gReceivePin = 11;
}

SoftwareSerial::~SoftwareSerial()
{
  stopListening();
}

void SoftwareSerial::init(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic)
{
  _receivePin = receivePin;
  _transmitPin = transmitPin;
  _inverse_logic = inverse_logic;
  gReceivePin = 11;
}

void SoftwareSerial::begin(long speed)
{
  if ((unsigned long)speed > MAX_BAUDRATE)
    speed = MAX_BAUDRATE;
  _baudRate = speed;
  _bitTimeUs = 1000000UL / _baudRate;

  setTX(_transmitPin);
  setRX(_receivePin);

  setupOEPins();

  // Setup RX port/mask
  _rxMask = digitalPinToBitMask(_receivePin);
  _rxPort = digitalPinToPort(_receivePin);

  // Setup TX port/mask
  _txMask = digitalPinToBitMask(_transmitPin);
  _txInvMask = ~_txMask;
  _txPort = digitalPinToPort(_transmitPin);

  attachInterrupt(digitalPinToInterrupt(gReceivePin), SoftwareSerial_GPIOTE_IRQHandler, FALLING);

  // Setup TIMER2 for 1 MHz, 32 bit mode
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER2->PRESCALER = TIMER_PRESCALER;
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

  NVIC_SetPriority(GPIOTE_IRQn, 0);
  NVIC_SetPriority(TIMER2_IRQn, 0);
  NVIC_EnableIRQ(TIMER2_IRQn);

  listen();
}

bool SoftwareSerial::listen()
{
  if (active_object != this)
  {
    if (active_object)
      active_object->stopListening();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    return true;
  }
  return false;
}

bool SoftwareSerial::stopListening()
{
  if (active_object == this)
  {
    active_object = nullptr;
    return true;
  }
  return false;
}

int SoftwareSerial::available()
{
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

int SoftwareSerial::read()
{
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;
  uint8_t d = _receive_buffer[_receive_buffer_head];
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

// Reads exactly 'length' bytes into buffer; blocks until all bytes received
void SoftwareSerial::readBytes(uint8_t *buffer, size_t length)
{
  size_t bytes_read = 0;
  while (bytes_read < length)
  {
    int c = read();
    if (c >= 0)
    {
      buffer[bytes_read] = (uint8_t)c;
      bytes_read++;
    }
    else
    {
      delayMicroseconds(100);
    }
  }
}

// Writes 'length' bytes from buffer; returns number of bytes written
size_t SoftwareSerial::writeBuffer(const uint8_t *buffer, size_t length)
{
  size_t bytes_written = 0;
  for (size_t i = 0; i < length; i++)
  {
    if (write(buffer[i]) == 1)
    {
      bytes_written++;
    }
    else
    {
      break; // error or busy
    }
  }
  return bytes_written;
}

// === WRITE ===
// TX with hardware-timed bits and direction control
size_t SoftwareSerial::write(uint8_t b)
{
  // Wait if previous TX still in progress
  while (_txInProgress)
  {
  }

  _txInProgress = true;
  _txByte = b;
  _txBitCount = 0;

  // Disable RX while transmitting
  _rxInProgress = false;

  // Enable TX driver buffer, disable RX driver buffer
  enableTXBuffer();

  // Drive start bit (line low)
  *((volatile uint32_t *)&_txPort->OUT) &= _txInvMask;

  // Reset and start TIMER2 for TX bit timing
  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->CC[0] = _bitTimeUs;
  NRF_TIMER2->TASKS_START = 1;

  // Wait until TX finishes (ISR clears _txInProgress)
  while (_txInProgress)
  {
  }

  return 1;
}

// === ISR implementations ===

// GPIOTE ISR: Detect start bit on RX line
extern "C" void SoftwareSerial_GPIOTE_IRQHandler() {
    // Disable RX GPIOTE interrupt while receiving
    NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_IN0_Msk;

    _rxInProgress = true;
    _rxBitCount = 0;
    _rxByte = 0;

    // Stop Timer2 and clear it
    NRF_TIMER2->TASKS_STOP = 1;
    NRF_TIMER2->TASKS_CLEAR = 1;
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;

    // Start Timer2 to sample first bit at 1.5 bit times
    NRF_TIMER2->CC[0] = _bitTimeUs + (_bitTimeUs / 2);
    NRF_TIMER2->TASKS_START = 1;
}

// TIMER2 ISR: Handle RX and TX bit timing
extern "C" void SoftwareSerial_TIMER2_IRQHandler(void) {
    // Clear the compare event immediately
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    // ------------------- RX handling -------------------
    if (_rxInProgress) {
        _rxByte >>= 1;
        if ((_rxPort->IN & _rxMask)) {
            _rxByte |= 0x80;
        }
        _rxBitCount++;

        if (_rxBitCount >= 8) {
            // Store byte in buffer
            uint8_t next = (SoftwareSerial::_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
            if (next != SoftwareSerial::_receive_buffer_head) {
                SoftwareSerial::_receive_buffer[SoftwareSerial::_receive_buffer_tail] = _rxByte;
                SoftwareSerial::_receive_buffer_tail = next;
            }

            _rxInProgress = false;

            // Stop Timer2 until next start bit
            NRF_TIMER2->TASKS_STOP = 1;
            NRF_TIMER2->TASKS_CLEAR = 1;

            // Re-enable GPIOTE interrupt for next start bit
            NRF_GPIOTE->EVENTS_IN[0] = 0;
            NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
        } else {
            // Schedule next bit sampling
            NRF_TIMER2->CC[0] += _bitTimeUs;
        }
    }

    // ------------------- TX handling -------------------
    if (_txInProgress) {
        _txBitCount++;

        if (_txBitCount <= 8) {
            if (_txByte & 0x01) {
                _txPort->OUT |= _txMask;
            } else {
                _txPort->OUT &= _txInvMask;
            }
            _txByte >>= 1;
            NRF_TIMER2->CC[0] += _bitTimeUs;
        } else if (_txBitCount == 9) {
            // Stop bit
            _txPort->OUT |= _txMask;
            NRF_TIMER2->CC[0] += _bitTimeUs;
        } else {
            _txInProgress = false;
            NRF_TIMER2->TASKS_STOP = 1;

            // Re-enable RX immediately
            enableRXBuffer();
            NRF_GPIOTE->EVENTS_IN[0] = 0;
            NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
        }
    }
}


// === Set TX pin function ===
void SoftwareSerial::setTX(uint8_t tx)
{
  digitalWrite(tx, HIGH); // idle high line
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  NRF_GPIO_Type *port = digitalPinToPort(tx);
  _transmitPortRegister = &(port->OUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  _txMask = digitalPinToBitMask(tx);
  _txInvMask = ~_txMask;
  _txPort = port;
  _transmitPin = tx;
}

// === Set RX pin function ===
void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT_PULLUP);
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  NRF_GPIO_Type *port = digitalPinToPort(rx);
  _receivePortRegister = &(port->IN);
  _rxMask = digitalPinToBitMask(rx);
  _rxPort = port;
}

void SoftwareSerial::flush()
{
  if (!isListening())
    return;

  NRF_GPIOTE->INTENCLR = _intMask; // Disable interrupt

  _receive_buffer_head = _receive_buffer_tail = 0; // Clear buffer

  NRF_GPIOTE->INTENSET = _intMask; // Enable interrupt
}