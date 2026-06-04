/**
 * @file    mavlinkLayer.h
 * @brief   MAVLink v2 message layer for mavlink-distance-node
 *
 * Provides the public interface for MAVLink message packing and transmission.
 * This layer has no knowledge of UART hardware, Zephyr scheduling, or
 * message queue internals. It depends only on the UART layer's
 * uart_queue_up() function for byte delivery.
 *
 * Defines the uart_packet_t structure used as the inter-layer
 * data transfer type between MAVLink and UART layers.
 *
 * @note All MAVLink packing uses c_library_v2 header-only library.
 *       No dynamic memory allocation is performed anywhere in this layer.
 */

#ifndef MAVLINKLAYER_H
#define MAVLINKLAYER_H

#include <stdint.h>
#include "common/mavlink.h"

/** @brief MAVLink system ID — unique node identifier on the MAVLink network */
#define SYS_ID           1U

/** @brief MAVLink component ID — identifies node as obstacle avoidance sensor */
#define COMP_ID          MAV_COMP_ID_OBSTACLE_AVOIDANCE

/** @brief Minimum valid sensor range in centimetres */
#define MIN_SENSOR_RANGE 50U

/** @brief Maximum valid sensor range in centimetres */
#define MAX_SENSOR_RANGE 200U

/**
 * @brief Serialised MAVLink packet ready for UART transmission
 *
 * Produced by mavlink_msg_to_send_buffer() and passed to the UART
 * layer via uart_queue_up(). The buf field contains a complete
 * MAVLink v2 frame including header, payload, and CRC.
 */
typedef struct {
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN]; /**< Serialised MAVLink frame bytes */
    uint16_t len;                          /**< Number of valid bytes in buf */
} uart_packet_t;

/**
 * @brief Pack and queue a MAVLink HEARTBEAT message
 *
 * Packs a HEARTBEAT identifying this node as MAV_TYPE_GENERIC with
 * MAV_AUTOPILOT_INVALID and MAV_STATE_ACTIVE. Serialises to a
 * uart_packet_t and passes to the UART layer queue.
 *
 * Must be called at 1 Hz or the flight controller will mark this
 * component as timed out and stop processing its sensor messages.
 */
void send_heart_beat(void);

/**
 * @brief Pack and queue all four directional DISTANCE_SENSOR messages
 *
 * Sends four MAVLink DISTANCE_SENSOR messages in sequence covering
 * front, right, back, and left directions. Each message carries an
 * independent distance value and the correct MAV_SENSOR_ROTATION
 * orientation for that direction.
 *
 * Sensor orientation mapping:
 * - Sensor 0: front  MAV_SENSOR_ROTATION_NONE
 * - Sensor 1: right  MAV_SENSOR_ROTATION_YAW_90
 * - Sensor 2: back   MAV_SENSOR_ROTATION_YAW_180
 * - Sensor 3: left   MAV_SENSOR_ROTATION_YAW_270
 *
 * @param frontSensorValue  Forward distance reading in centimetres
 * @param rightSensorValue  Right distance reading in centimetres
 * @param backSensorValue   Rear distance reading in centimetres
 * @param leftSensorValue   Left distance reading in centimetres
 *
 * @note All values must be between MIN_SENSOR_RANGE and MAX_SENSOR_RANGE
 *       or the flight controller may reject the reading
 */
void send_all_distance_data(uint16_t frontSensorValue,
                             uint16_t rightSensorValue,
                             uint16_t backSensorValue,
                             uint16_t leftSensorValue);

#endif
