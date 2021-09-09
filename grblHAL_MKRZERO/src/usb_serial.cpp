/*

  usb_serial.cpp - USB serial port wrapper for Arduino MKRZERO

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>

#include "Arduino.h"

#include "driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "grbl/protocol.h"

#define BLOCK_RX_BUFFER_SIZE 20

static stream_rx_buffer_t rxbuf;
static stream_block_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

//
// Returns number of characters in serial input buffer
//
static uint16_t usbRxCount (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;

    return (uint16_t)BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
static uint16_t usbRxFree (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;

    return (uint16_t)((RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE));
}

//
// Flushes the serial input buffer (including the USB buffer)
//
static void usbRxFlush (void)
{
    SerialUSB.flush();
    rxbuf.overflow = Off;
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void usbRxCancel (void)
{
    rxbuf.data[rxbuf.head] = CMD_RESET;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = (rxbuf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Writes a character to the serial output stream
//
static bool usbPutC (const char c)
{
    SerialUSB.write(c);

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full.
// Buffers locally up to 40 characters or until the string is terminated with a ASCII_LF character.
// NOTE: grbl always sends ASCII_LF terminated strings!
//
static void usbWriteS (const char *s)
{
    size_t length = strlen(s);

    if((length + txbuf.length) < BLOCK_TX_BUFFER_SIZE) {

        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;

        if(s[length - 1] == ASCII_LF || txbuf.length > txbuf.max_length) {

            size_t avail;
            txbuf.s = txbuf.data;

            while(txbuf.length) {

                if((avail = SerialUSB.availableForWrite()) > 10) {

                    length = avail < txbuf.length ? avail : txbuf.length;

                    SerialUSB.write((uint8_t *)txbuf.s, length); // doc is wrong - does not return bytes sent!

                    txbuf.length -= length;
                    txbuf.s += length;
                }

                if(txbuf.length && !hal.stream_blocking_callback())
                    return;
            }
            txbuf.length = 0;
            txbuf.s = txbuf.data;
        }
    }
}

//
// Writes a null terminated string to the serial output stream followed by EOL, blocks if buffer full
//
static void usbWriteLn (const char *s)
{
    usbWriteS(s);
    usbWriteS(ASCII_EOL);
}

//
// Writes a number of characters from string to the serial output stream, blocks if buffer full
//
static void usbWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        usbPutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t usbGetC (void)
{
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available else EOF

    char data = rxbuf.data[bptr++];              // Get next character, increment tmp pointer
    rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return (int16_t)data;
}

static bool usbSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static bool usb_serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usbSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *usbInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .connected = false,
        .get_rx_buffer_free = usbRxFree,
        .write = usbWriteS,
        .write_all = usbWriteS,
        .write_char = usbPutC,
        .enqueue_rt_command = usb_serialEnqueueRtCommand,
        .read = usbGetC,
        .reset_read_buffer = usbRxFlush,
        .cancel_read_buffer = usbRxCancel,
        .set_enqueue_rt_handler = usbSetRtHandler,
        .suspend_read = usbSuspendInput
    };

    SerialUSB.begin(BAUD_RATE);

#if usb_WAIT
//    while(!SerialUSB); // Hangs forever...
#endif

    txbuf.s = txbuf.data;
    txbuf.max_length = SerialUSB.availableForWrite(); // 63 bytes
    txbuf.max_length = (txbuf.max_length > BLOCK_TX_BUFFER_SIZE ? BLOCK_TX_BUFFER_SIZE : txbuf.max_length) - 20;

    return &stream;
}


//
// This function get called from the protocol_execute_realtime function,
// used here to get characters off the USB serial input stream and buffer
// them for processing by grbl. Real time command characters are stripped out
// and submitted for realtime processing.
//
void usb_execute_realtime (uint_fast16_t state)
{
    char c, *dp;
    int avail, free;
    static char tmpbuf[BLOCK_RX_BUFFER_SIZE];

    if((avail = SerialUSB.available())) {

        dp = tmpbuf;
        free = usbRxFree();
        free = free > BLOCK_RX_BUFFER_SIZE ? BLOCK_RX_BUFFER_SIZE : free;
        avail = avail > free ? free : avail;

        SerialUSB.readBytes(tmpbuf, avail);

        while(avail--) {
            c = *dp++;
            if(!enqueue_realtime_command(c)) {                      // Check and strip realtime commands...
                uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
                if(next_head == rxbuf.tail)                         // If buffer full
                    rxbuf.overflow = On;                            // flag overflow,
                else {
                    rxbuf.data[rxbuf.head] = c;                     // else add character data to buffer
                    rxbuf.head = next_head;                         // and update pointer
                }
            }
        }
    }
}

#ifdef __cplusplus
}
#endif
