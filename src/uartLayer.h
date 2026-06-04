/**
 * @file    uartLayer.h
 * @brief   UART transmission layer for mavlink-distance-node
 *
 * Provides the public interface for UART initialisation and packet queuing.
 * Internally owns a Zephyr message queue and a dedicated TX thread that
 * drains the queue and transmits bytes via uart_poll_out(). These
 * implementation details are private to uartLayer.c and not exposed here.
 *
 * This layer uses the Zephyr UART driver API exclusively — no register
 * level hardware access. Compatible with any board supported by Zephyr
 * that exposes a usart2 device tree node.
 *
 * @note uart_packet_t is defined in mavlinkLayer.h and shared between
 *       the MAVLink and UART layers as the inter-layer data transfer type.
 */

#ifndef UARTLAYER_H
#define UARTLAYER_H

#include <stdint.h>
#include "mavlinkLayer.h"
#include <zephyr/kernel.h>

/**
 * @brief Initialise UART2 with MAVLink communication settings
 *
 * Configures UART2 via the Zephyr UART driver API:
 * - Baud rate: 57600
 * - Data bits: 8
 * - Parity: none
 * - Stop bits: 1
 * - Flow control: none
 *
 * Must be called once before any call to uart_queue_up().
 *
 * @return 0 on success, negative errno on failure
 */
int uart_init(void);

/**
 * @brief Queue a serialised MAVLink packet for UART transmission
 *
 * Copies the packet into the internal message queue. The internal TX thread
 * drains the queue and transmits bytes asynchronously. This function returns
 * immediately without blocking the caller.
 *
 * If the queue is full the packet is dropped. Queue depth is 100 packets
 * which is sufficient for normal 1Hz heartbeat and 10Hz sensor operation.
 *
 * @param uartPkt  Pointer to uart_packet_t containing serialised MAVLink frame
 * @return         0 on success, negative value if queue put failed
 *
 * @note This is the only UART layer function called by the MAVLink layer
 */
int uart_queue_up(uart_packet_t *uartPkt);

/**
 * @brief Transmit a serialised MAVLink packet over UART2
 *
 * Blocking byte-by-byte transmission using uart_poll_out(). Called
 * internally by the UART TX thread — not intended for direct use by
 * other layers.
 *
 * @param uartPkt  uart_packet_t containing bytes and length to transmit
 * @return         0 on success
 */
int uart_send(uart_packet_t uartPkt);

#endif
