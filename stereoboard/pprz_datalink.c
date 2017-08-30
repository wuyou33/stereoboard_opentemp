/*
 * datalink.c
 *
 *  Created on: 22 Aug 2017
 *      Author: kirk
 */

#include "pprz_datalink.h"

struct pprz_transport pprz;
struct link_device dev;

/* Send completed datalink message
 *
 */
static void send_message(void *port, long fd)
{
}

/* Put single byte in datalink
 * Defaults to the UART datalink
 *
 */
static void put_byte(void *dev, long fd, uint8_t data)
{
  while (((struct UartDataStruct*)dev)->tx(&data, 1) == 0)
    ;
}

/* Add data buffer to datalink
 * @param dev Device to send message
 * @param fd
 * @param data Data array to send
 * @param length Length of data array pointed to by data
 */
static void put_buffer(void *dev, long fd, uint8_t *data, uint16_t length)
{
  uint16_t i;
  for (i = 0; i < length; i++) {
    put_byte(dev, fd, data[i]);
  }
}

static int check_free_space(void *dev, long *fd, uint16_t length)
{
  struct UartDataStruct* device = dev;
  return ((((device->usart_tx_counter_read - device->usart_tx_counter_write) - 1)
      + TXBUFFERSIZE) % TXBUFFERSIZE);
}

static int char_available(void *dev)
{
  return usart_char_available((struct UartDataStruct *)dev);
}

static uint8_t get_byte(void *dev)
{
  return (((struct UartDataStruct*)dev)->rx());
}

/* Initialise the datalink
 * @param periph Communication peripheral
 *
 */
void datalink_init(struct UartDataStruct* periph)
{
  pprz_transport_init(&pprz);
  dev.periph = periph; // dafault uart port

  // tx
  dev.put_byte = (put_byte_t)put_byte;
  dev.put_buffer = (put_buffer_t)put_buffer;
  dev.send_message = (send_message_t)send_message;
  dev.check_free_space = (check_free_space_t)check_free_space;

  // rx
  dev.char_available = (char_available_t)char_available;
  dev.get_byte = (get_byte_t)get_byte;
}
