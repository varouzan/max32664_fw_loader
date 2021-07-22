/*
 * i2cBitBangingBus.cpp
 *
 *  Created on: 06.03.2015
 *      Author: "Marek Wyborski"
 */

#include "i2cBitBangingBus.h"

#include <wiringPi.h>

#include <iostream>
#include <memory>
#include <stdexcept>

i2cBitBangingBus::i2cBitBangingBus(uint8_t pin_number_sda, uint8_t pin_number_scl, uint32_t sleepTimeNanos_,
        uint32_t delayTicks_) :
        PIN_SDA(pin_number_sda), PIN_SCL(pin_number_scl), sleepTimeNanos(sleepTimeNanos_), nanoSleepTime(), delayTicks(
                delayTicks_), i2c_started(false)
{
    // Pull up setzen 50KΩ
    // http://wiringpi.com/reference/core-functions/
//    pullUpDnControl(PIN_SDA,PUD_OFF);
//    pullUpDnControl(PIN_SCL,PUD_OFF);

    nanoSleepTime.tv_sec = 0;
    nanoSleepTime.tv_nsec = 1;

}



// I2C implementation is copied and pasted from wikipedia:
// 
// https://en.wikipedia.org/wiki/I%C2%B2C#Example_of_bit-banging_the_I.C2.B2C_master_protocol
//
//



bool i2cBitBangingBus::read_SCL() // Set SCL as input and return current level of line, 0 or 1
{
    pinMode(PIN_SCL, INPUT);
    return digitalRead(PIN_SCL);
}

bool i2cBitBangingBus::read_SDA() // Set SDA as input and return current level of line, 0 or 1
{
    pinMode(PIN_SDA, INPUT);
    return digitalRead(PIN_SDA);
}

void i2cBitBangingBus::clear_SCL() // Actively drive SCL signal low
{
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SCL, 0);
}

void i2cBitBangingBus::clear_SDA() // Actively drive SDA signal low
{
    pinMode(PIN_SDA, OUTPUT);
    digitalWrite(PIN_SDA, 0);
}

void i2cBitBangingBus::arbitration_lost(string where)
{
    throw runtime_error("Connection lost: " + where);
}

void i2cBitBangingBus::i2c_sleep()
{
    if (sleepTimeNanos)
#ifdef NO_NANOSLEEP
        usleep(sleepTimeNanos / 1000);
#else
        nanosleep(&nanoSleepTime, NULL);
#endif
}

void i2cBitBangingBus::i2c_delay()
{
    unsigned int index;
    for (index = 0; index < delayTicks; index++)
        ;
}

void i2cBitBangingBus::i2c_start_cond()
{
    if (i2c_started)
    { // if started, do a restart cond
      // set SDA to 1
        read_SDA();
        i2c_delay();
        while (read_SCL() == 0)
        {  // Clock stretching
            i2c_sleep();
        }
        // Repeated start setup time, minimum 4.7us
        i2c_delay();
    }
    if (read_SDA() == 0)
    {
        arbitration_lost("i2c_start_cond");
    }
    // SCL is high, set SDA from 1 to 0.
    clear_SDA();
    i2c_delay();
    clear_SCL();
    i2c_started = true;
}

void i2cBitBangingBus::i2c_stop_cond(void)
{
    // set SDA to 0
    clear_SDA();
    i2c_delay();
    // Clock stretching
    while (read_SCL() == 0)
    {
        // add timeout to this loop.
        i2c_sleep();
    }
    // Stop bit setup time, minimum 4us
    i2c_delay();
//  usleep(4);
    read_SDA();
    // SCL is high, set SDA from 0 to 1
    if (read_SDA() == 0)
    {
        arbitration_lost("i2c_stop_cond");
    }
    i2c_delay();
    i2c_started = false;
}

// Write a bit to I2C bus
void i2cBitBangingBus::i2c_write_bit(bool bit)
{
    if (bit)
    {
        read_SDA();
    }
    else
    {
        clear_SDA();
    }
    i2c_delay();
    while (read_SCL() == 0)
    { // Clock stretching
      // You should add timeout to this loop
        i2c_sleep();
    }
    // SCL is high, now data is valid
    // If SDA is high, check that nobody else is driving SDA
    if (bit && read_SDA() == 0)
    {
        arbitration_lost("i2c_write_bit");
    }
    i2c_delay();
    clear_SCL();
}

// Read a bit from I2C bus
bool i2cBitBangingBus::i2c_read_bit()
{
    bool bit;
    // Let the slave drive data
    read_SDA();
    i2c_delay();
    while (read_SCL() == 0)
    { // Clock stretching
      // You should add timeout to this loop
        i2c_sleep();
    }
    // SCL is high, now data is valid
    bit = read_SDA();
    i2c_delay();
    clear_SCL();

//  cout << "Bit: " << (bit ? "1" : "0" )<< endl;

    return bit;
}

// Write a byte to I2C bus. Return 0 if ack by the slave.
bool i2cBitBangingBus::i2c_write_byte(bool send_start, bool send_stop, uint8_t byte)
{
    unsigned bit;
    bool nack;
    if (send_start)
    {
        i2c_start_cond();
    }
    for (bit = 0; bit < 8; bit++)
    {
        i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    nack = i2c_read_bit();
    if (send_stop)
    {
        i2c_stop_cond();
    }
    return nack;
}

// Read a byte from I2C bus
uint8_t i2cBitBangingBus::i2c_read_byte(bool nack, bool send_stop)
{
    unsigned char byte = 0;
    unsigned bit;
    for (bit = 0; bit < 8; bit++)
    {
        byte = (byte << 1) | i2c_read_bit();
    }
    i2c_write_bit(nack);
    if (send_stop)
    {
        i2c_stop_cond();
    }
    return byte;
}

// KERNEL-LIKE I2C METHODS

// This executes the SMBus “write byte” protocol, returning negative errno else zero on success.
int32_t i2cBitBangingBus::i2c_smbus_write_byte_data(uint8_t i2c_address, uint8_t command, uint8_t value)
{
    // 7 bit address + 1 bit read/write
    // read = 1, write = 0
    // http://www.totalphase.com/support/articles/200349176-7-bit-8-bit-and-10-bit-I2C-Slave-Addressing
    uint8_t address = (i2c_address << 1) | 0;

    if (!i2c_write_byte(true, false, address))
    {
        if (!i2c_write_byte(false, false, command))
        {
            if (!i2c_write_byte(false, true, value))
            {
                return 0;
            }
        }
        else
            i2c_stop_cond();
    }
    else
        i2c_stop_cond();

    return -1;
}

// This executes the SMBus “read byte” protocol, returning negative errno else a data byte received from the device.
int32_t i2cBitBangingBus::i2c_smbus_read_byte_data(uint8_t i2c_address, uint8_t command)
{
    uint8_t address = (i2c_address << 1) | 0;
    if (!i2c_write_byte(true, false, address))
    {

        if (!i2c_write_byte(false, false, command))
        {

            address = (i2c_address << 1) | 1;
            if (!i2c_write_byte(true, false, address))
            {
                return i2c_read_byte(true, true);
            }
            else
                i2c_stop_cond();
        }
        else
            i2c_stop_cond();
    }
    else
        i2c_stop_cond();

    return -1;
}

// This executes the SMBus “block write” protocol, returning negative errno else zero on success.
int32_t i2cBitBangingBus::i2c_smbus_write_i2c_block_data(uint8_t i2c_address, uint8_t command, uint8_t length,
        const uint8_t * values)
{
    // 7 bit address + 1 bit read/write
    // read = 1, write = 0
    // http://www.totalphase.com/support/articles/200349176-7-bit-8-bit-and-10-bit-I2C-Slave-Addressing
    uint8_t address = (i2c_address << 1) | 0;

    if (!i2c_write_byte(true, false, address))
    {
        if (!i2c_write_byte(false, false, command))
        {
            bool errors = false;
            for (size_t i = 0; i < length; i++)
            {
                if (!errors)
                {
                    errors = i2c_write_byte(false, false, values[i]);
                }
            }

            i2c_stop_cond();

            if (!errors)
                return 0;
        }
        else
            i2c_stop_cond();
    }
    else
        i2c_stop_cond();

    return -1;
}

//
int32_t i2cBitBangingBus::i2c_smbus_write_i2c_block_data_no_command(uint8_t i2c_address, uint32_t length,
        const uint8_t * values)
{
    // 7 bit address + 1 bit read/write
    // read = 1, write = 0
    // http://www.totalphase.com/support/articles/200349176-7-bit-8-bit-and-10-bit-I2C-Slave-Addressing
    uint8_t address = (i2c_address << 1) | 0;
    int counter=0;

    if (!i2c_write_byte(true, false, address))
    {
        bool errors = false;
        for (size_t i = 0; i < length; i++)
        {
            if (!errors)
            {
                errors = i2c_write_byte(false, false, values[i]);
                //printf("%x ",values[i]);
                counter++;
            }
        }

        i2c_stop_cond();

        if (!errors)
            std::cout<<"   WROTE "<<counter<<endl;
            return 0;
    }
    else
        i2c_stop_cond();

    return -1;
}

// This executes the SMBus “block read” protocol, returning negative errno else the number
// of data bytes in the slave's response.
int32_t i2cBitBangingBus::i2c_smbus_read_i2c_block_data(uint8_t i2c_address, uint8_t command, uint8_t length,
        uint8_t* values)
{
    uint8_t address = (i2c_address << 1) | 0;
    if (!i2c_write_byte(true, false, address))
    {

        if (!i2c_write_byte(false, false, command))
        {

            address = (i2c_address << 1) | 1;
            if (!i2c_write_byte(true, false, address))
            {
                for (uint8_t i = 0; i < length; i++)
                {
                    values[i] = i2c_read_byte(i == (length - 1), i == (length - 1));
                }

                return length;
            }
            else
                i2c_stop_cond();
        }
        else
            i2c_stop_cond();
    }
    else
        i2c_stop_cond();

    return -1;
}

//
int32_t i2cBitBangingBus::i2c_smbus_read_i2c_block_data_no_command(uint8_t i2c_address, uint8_t length,
        uint8_t* values)
{
    uint8_t address; 
    address = (i2c_address << 1) | 1;
    if (!i2c_write_byte(true, false, address))
    {
        for (uint8_t i = 0; i < length; i++)
        {
            values[i] = i2c_read_byte(i == (length - 1), i == (length - 1));
        }

        return length;
    }
    else
        i2c_stop_cond();


    return -1;
}
