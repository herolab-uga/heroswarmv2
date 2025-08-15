#ifndef SOFTWARESERIAL_HPP
#define SOFTWARESERIAL_HPP

#include <stdint.h>
#include <stddef.h>

class SoftwareSerial {
public:
    SoftwareSerial();
    SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
    virtual ~SoftwareSerial();

    void init(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic);

    virtual size_t write(uint8_t byte);
    virtual size_t writeBuffer(const uint8_t* buffer, size_t length);

    virtual int read();
    virtual void readBytes(uint8_t* buffer, size_t length);

    virtual int available();
    virtual void flush();

    void begin(long speed);
    void end();

    bool listen();
    bool stopListening();

    // Buffer
    static const uint8_t _SS_MAX_RX_BUFF = 64;
    static char _receive_buffer[_SS_MAX_RX_BUFF];
    static volatile uint8_t _receive_buffer_tail;
    static volatile uint8_t _receive_buffer_head;

private:
    void recv();
    uint32_t rx_pin_read();

    bool inverse_logic = false;

    // Pins
    uint8_t _receivePin = 0;
    uint8_t _transmitPin = 0;

    // Delays in microseconds
    uint16_t _rx_delay_centering = 0;
    uint16_t _rx_delay_intrabit = 0;
    uint16_t _rx_delay_stopbit = 0;
    uint16_t _tx_delay = 0;

    bool _inverse_logic = false;
    bool _buffer_overflow = false;

    // Interrupt mask
    uint32_t _intMask = 0;

    static SoftwareSerial* active_object;

    static void handle_interrupt();

    void setTX(uint8_t tx);
    void setRX(uint8_t rx);
    bool isListening() { return active_object == this; }
};

#endif // SOFTWARESERIAL_HPP
