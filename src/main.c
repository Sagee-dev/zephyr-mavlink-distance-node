/**
 * @mainpage mavlink-distance-node
 *
 * @section intro Introduction
 * A portable MAVLink v2 fake distance sensor node implemented on Zephyr RTOS.
 * Simulates four directional distance sensors (front, right, back, left) and
 * transmits readings to a flight controller over UART using standard MAVLink
 * DISTANCE_SENSOR messages.
 *
 * Designed to be platform-agnostic — any board supported by Zephyr with a
 * UART peripheral can run this node without hardware modification.
 *
 * @section arch Software Architecture
 * The project is structured in three independent layers:
 *
 * - Application Layer (main.c) — scheduling, sensor simulation, work items
 * - MAVLink Layer (mavlinkLayer.c) — message packing and serialisation
 * - UART Layer (uartLayer.c) — transmission, message queue, TX thread
 *
 * @section scheduling Scheduling
 * Zephyr delayable work items replace dedicated threads for the application
 * logic. Both heartbeat and distance work items run on the system workqueue
 * and reschedule themselves after each execution. No mutex is required because
 * the system workqueue is single-threaded by default.
 *
 * A dedicated UART TX thread inside the UART layer drains the message queue
 * and transmits packets independently of the application work items.
 *
 * @section portability Portability
 * No hardware-specific register access is used anywhere in the application or
 * MAVLink layers. The UART layer uses the Zephyr UART driver API exclusively.
 * To port to a different board, update the board overlay and prj.conf only.
 *
 * @section msgs MAVLink Messages
 * | Message            | Rate  | Description                        |
 * |--------------------|-------|------------------------------------|
 * | HEARTBEAT          | 1 Hz  | Node identification to FC          |
 * | DISTANCE_SENSOR x4 | 10 Hz | Front, right, back, left readings  |
 *
 * @section author Author
 * W S G De Silva
 * M.Sc. Embedded Systems, Technische Universitat Chemnitz
 *
 * @section date Date
 * 2026
 */

/**
 * @file    main.c
 * @brief   Application layer — sensor simulation and work item scheduling
 *
 * Implements the top-level application logic for the mavlink-distance-node.
 * Defines the distanceSensor_t structure, initialises four sensor instances
 * with independent speeds, and schedules MAVLink transmissions using Zephyr
 * delayable work items.
 *
 * No UART or MAVLink protocol details are present in this layer.
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/random/random.h>

#include <stdio.h>
#include <string.h>

#include "uartLayer.h"
#include "mavlinkLayer.h"

/** @brief Minimum valid sensor distance in centimetres */
#define MIN_DIST_CM 50U

/** @brief Maximum valid sensor distance in centimetres */
#define MAX_DIST_CM 200U

/** @brief Zephyr delayable work item for HEARTBEAT transmission */
static struct k_work_delayable heartbeat_dwork;

/** @brief Zephyr delayable work item for DISTANCE_SENSOR transmission */
static struct k_work_delayable distance_dwork;

/**
 * @brief Simulated distance sensor state
 *
 * Tracks the current distance value and triangle wave oscillation state
 * for one simulated sensor. Each sensor instance has an independent speed
 * producing visibly different oscillation rates on the flight controller.
 */
typedef struct {
    uint8_t  id;        /**< Sensor instance identifier 0-3 */
    int16_t  distance;  /**< Current distance value in centimetres */
    uint16_t speed;     /**< Step size in centimetres per update cycle */
    int16_t  direction; /**< Oscillation direction: +1 increment, -1 decrement */
} distanceSensor_t;

/**
 * @brief Sensor array — four directional sensors with independent speeds
 *
 * Sensor 0: front,  starts at 50 cm,  speed 2 cm/cycle  — slow
 * Sensor 1: right,  starts at 80 cm,  speed 5 cm/cycle  — medium
 * Sensor 2: back,   starts at 120 cm, speed 8 cm/cycle  — fast
 * Sensor 3: left,   starts at 160 cm, speed 3 cm/cycle  — medium slow
 */
distanceSensor_t sensorArr[4] = {
    {0, 50,  2, 1},
    {1, 80,  5, 1},
    {2, 120, 8, 1},
    {3, 160, 3, 1},
};

/**
 * @brief Update sensor state and return current distance value
 *
 * Advances the sensor triangle wave by one step. When the value reaches
 * MAX_DIST_CM the direction reverses to decrement. When it reaches
 * MIN_DIST_CM the direction reverses to increment.
 *
 * @param sensor  Pointer to sensor state to update
 * @return        Current distance value in centimetres after update
 *
 * @note Called once per 100ms cycle per sensor from distance_handler
 */
uint16_t fake_distance(distanceSensor_t *sensor) {
    sensor->distance += (sensor->direction * sensor->speed);
    if (sensor->distance >= MAX_DIST_CM) {
        sensor->distance = MAX_DIST_CM;
        sensor->direction = -1;
        return sensor->distance;
    }
    if (sensor->distance <= MIN_DIST_CM) {
        sensor->distance = MIN_DIST_CM;
        sensor->direction = 1;
        return sensor->distance;
    }
    return sensor->distance;
}

/**
 * @brief Zephyr work handler — transmit HEARTBEAT and reschedule
 *
 * Called by the system workqueue every 1000ms. Sends one MAVLink
 * HEARTBEAT message then reschedules itself for the next cycle.
 *
 * @param work  Pointer to the work item (unused, required by Zephyr API)
 */
static void heartbeat_handler(struct k_work *work) {
    send_heart_beat();
    k_work_schedule(&heartbeat_dwork, K_MSEC(1000));
}

/**
 * @brief Zephyr work handler — transmit all four sensor readings and reschedule
 *
 * Called by the system workqueue every 100ms. Updates all four sensor
 * simulation states, then sends four MAVLink DISTANCE_SENSOR messages.
 * Reschedules itself for the next cycle.
 *
 * @param work  Pointer to the work item (unused, required by Zephyr API)
 */
static void distance_handler(struct k_work *work) {
    send_all_distance_data(
        fake_distance(&sensorArr[0]),
        fake_distance(&sensorArr[1]),
        fake_distance(&sensorArr[2]),
        fake_distance(&sensorArr[3])
    );
    k_work_schedule(&distance_dwork, K_MSEC(100));
}

/**
 * @brief Application entry point
 *
 * Initialises the UART layer, registers both delayable work items,
 * and schedules the first execution of each. Returns immediately after
 * scheduling — Zephyr kernel takes over from this point.
 *
 * @return 0 on success
 */
int main(void) {
    uart_init();

    k_work_init_delayable(&heartbeat_dwork, heartbeat_handler);
    k_work_init_delayable(&distance_dwork,  distance_handler);

    k_work_schedule(&heartbeat_dwork, K_MSEC(1000));
    k_work_schedule(&distance_dwork,  K_MSEC(100));

    return 0;
}
