/**
 * @file    uartLayer.c
 * @brief   UART transmission layer implementation
 *
 * Implements UART initialisation, message queuing, and byte transmission
 * for the mavlink-distance-node. Owns a Zephyr message queue and a
 * dedicated TX thread that drains the queue independently of the
 * application and MAVLink layers.
 *
 * The message queue and TX thread are private to this file. External
 * callers interact only via uart_init() and uart_queue_up().
 *
 * No mutex is required because the system workqueue that drives the
 * MAVLink layer is single-threaded by default in Zephyr. The queue
 * itself is thread-safe by design.
 *
 * Platform portability: no register-level access is used. All hardware
 * interaction goes through the Zephyr UART driver API. To port to a
 * different board update the device tree node label and board overlay.
 */

#include "uartLayer.h"
#include "mavlinkLayer.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <string.h>

/** @brief Zephyr UART device handle — resolved at compile time from device tree */
const struct device *uart2 = DEVICE_DT_GET(DT_NODELABEL(usart2));

/**
 * @brief Internal UART transmit message queue
 *
 * Holds serialised MAVLink packets pending transmission. Depth of 100
 * packets provides sufficient headroom for normal 1Hz and 10Hz operation
 * with no packet loss under any scheduling condition.
 *
 * Private to this translation unit — not accessible from other layers.
 */
K_MSGQ_DEFINE(uart_msgq, sizeof(uart_packet_t), 100, 4);

int uart_init(void) {
    struct uart_config uart_cfg = {
        .baudrate  = 57600,
        .parity    = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        .data_bits = UART_CFG_DATA_BITS_8,
    };

    int ret = uart_configure(uart2, &uart_cfg);
    return ret;
}

int uart_send(uart_packet_t uartPkt) {
    for (int i = 0; i < uartPkt.len; i++) {
        uart_poll_out(uart2, uartPkt.buf[i]);
    }
    return 0;
}

int uart_queue_up(uart_packet_t *uartPkt) {
    return k_msgq_put(&uart_msgq, uartPkt, K_NO_WAIT);
}

/**
 * @brief Internal UART TX thread function
 *
 * Blocks on the internal message queue indefinitely. When a packet arrives
 * it is dequeued and transmitted byte by byte via uart_send(). Runs at
 * priority 3 — higher than the system workqueue default — to ensure
 * packets are drained promptly without queue overflow.
 *
 * This function is private to the UART layer. It is not declared in the
 * header and is not callable from other layers.
 *
 * @param a  Unused thread argument
 * @param b  Unused thread argument
 * @param c  Unused thread argument
 */
static void uart_tx_thread(void *a, void *b, void *c) {
    while (1) {
        uart_packet_t pkt;
        k_msgq_get(&uart_msgq, &pkt, K_FOREVER);
        uart_send(pkt);
    }
}

/** @brief Static TX thread definition — started automatically by Zephyr kernel */
K_THREAD_DEFINE(uart_tx_tid, 2048, uart_tx_thread, NULL, NULL, NULL, 3, 0, 0);
