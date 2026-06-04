/**
 * @file    mavlinkLayer.c
 * @brief   MAVLink v2 message packing and transmission implementation
 *
 * Implements HEARTBEAT and DISTANCE_SENSOR message generation using the
 * MAVLink c_library_v2 header-only library. Each function packs a message
 * into a uart_packet_t buffer and passes it to the UART layer queue via
 * uart_queue_up(). This layer has no knowledge of UART registers, Zephyr
 * driver internals, or queue implementation details.
 *
 * Uses k_uptime_get_32() for the MAVLink time_boot_ms timestamp field,
 * providing millisecond resolution uptime sourced from the Zephyr kernel.
 */

#include "mavlinkLayer.h"
#include "common/mavlink.h"
#include "zephyr/kernel.h"
#include "uartLayer.h"

void send_heart_beat(void) {
    mavlink_message_t msg;
    uart_packet_t hearBeatPacket;

    mavlink_msg_heartbeat_pack(
        SYS_ID, COMP_ID, &msg,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_INVALID,
        0, 0,
        MAV_STATE_ACTIVE
    );

    uint16_t len = mavlink_msg_to_send_buffer(hearBeatPacket.buf, &msg);
    hearBeatPacket.len = len;
    uart_queue_up(&hearBeatPacket);
}

/**
 * @brief Pack and queue a single DISTANCE_SENSOR message
 *
 * Internal helper called four times by send_all_distance_data() with
 * different sensor IDs, orientations, and distance values. Packs a
 * complete MAVLink DISTANCE_SENSOR frame and queues it for transmission.
 *
 * The quaternion field is set to identity [1, 0, 0, 0] indicating no
 * additional mounting angle correction beyond the orientation enum value.
 * Covariance is set to 255 indicating unknown measurement uncertainty.
 * Signal quality is set to 0 indicating unknown.
 *
 * @param id           Sensor instance number 0-3
 * @param orientation  MAV_SENSOR_ROTATION_* enum value for this sensor
 * @param distance     Current distance reading in centimetres
 */
static void send_fake_distance(uint8_t id, uint8_t orientation, uint16_t distance) {
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    mavlink_message_t msg;
    uart_packet_t distancePacket;

    mavlink_msg_distance_sensor_pack(
        SYS_ID,
        COMP_ID,
        &msg,
        k_uptime_get_32(),              /**< Milliseconds since boot — Zephyr kernel time */
        MIN_SENSOR_RANGE,               /**< Sensor minimum range in centimetres */
        MAX_SENSOR_RANGE,               /**< Sensor maximum range in centimetres */
        distance,                       /**< Current simulated distance in centimetres */
        MAV_DISTANCE_SENSOR_LASER,      /**< Sensor type — laser rangefinder */
        id,                             /**< Sensor instance identifier */
        orientation,                    /**< MAV_SENSOR_ROTATION_* mounting direction */
        255,                            /**< Covariance — 255 = unknown */
        0.0f,                           /**< Horizontal FOV — 0 = unknown */
        0.0f,                           /**< Vertical FOV — 0 = unknown */
        q,                              /**< Quaternion — identity, no additional rotation */
        0                               /**< Signal quality — 0 = unknown */
    );

    uint16_t len = mavlink_msg_to_send_buffer(distancePacket.buf, &msg);
    distancePacket.len = len;
    uart_queue_up(&distancePacket);
}

void send_all_distance_data(uint16_t frontSensorValue,
                             uint16_t rightSensorValue,
                             uint16_t backSensorValue,
                             uint16_t leftSensorValue) {
    send_fake_distance(0, MAV_SENSOR_ROTATION_NONE,    frontSensorValue);
    send_fake_distance(1, MAV_SENSOR_ROTATION_YAW_90,  rightSensorValue);
    send_fake_distance(2, MAV_SENSOR_ROTATION_YAW_180, backSensorValue);
    send_fake_distance(3, MAV_SENSOR_ROTATION_YAW_270, leftSensorValue);
}
