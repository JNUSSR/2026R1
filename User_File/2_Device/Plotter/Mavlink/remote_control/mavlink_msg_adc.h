#pragma once
// MESSAGE adc PACKING

#define MAVLINK_MSG_ID_adc 202


typedef struct __mavlink_adc_t {
 uint16_t adc1; /*<  adc1 value*/
 uint16_t adc2; /*<  adc2 value*/
 uint16_t adc3; /*<  adc3 value*/
} mavlink_adc_t;

#define MAVLINK_MSG_ID_adc_LEN 6
#define MAVLINK_MSG_ID_adc_MIN_LEN 6
#define MAVLINK_MSG_ID_202_LEN 6
#define MAVLINK_MSG_ID_202_MIN_LEN 6

#define MAVLINK_MSG_ID_adc_CRC 135
#define MAVLINK_MSG_ID_202_CRC 135



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_adc { \
    202, \
    "adc", \
    3, \
    {  { "adc1", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_adc_t, adc1) }, \
         { "adc2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_adc_t, adc2) }, \
         { "adc3", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_adc_t, adc3) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_adc { \
    "adc", \
    3, \
    {  { "adc1", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_adc_t, adc1) }, \
         { "adc2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_adc_t, adc2) }, \
         { "adc3", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_adc_t, adc3) }, \
         } \
}
#endif

/**
 * @brief Pack a adc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param adc1  adc1 value
 * @param adc2  adc2 value
 * @param adc3  adc3 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t adc1, uint16_t adc2, uint16_t adc3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_adc_LEN];
    _mav_put_uint16_t(buf, 0, adc1);
    _mav_put_uint16_t(buf, 2, adc2);
    _mav_put_uint16_t(buf, 4, adc3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_adc_LEN);
#else
    mavlink_adc_t packet;
    packet.adc1 = adc1;
    packet.adc2 = adc2;
    packet.adc3 = adc3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_adc_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_adc;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
}

/**
 * @brief Pack a adc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param adc1  adc1 value
 * @param adc2  adc2 value
 * @param adc3  adc3 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t adc1, uint16_t adc2, uint16_t adc3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_adc_LEN];
    _mav_put_uint16_t(buf, 0, adc1);
    _mav_put_uint16_t(buf, 2, adc2);
    _mav_put_uint16_t(buf, 4, adc3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_adc_LEN);
#else
    mavlink_adc_t packet;
    packet.adc1 = adc1;
    packet.adc2 = adc2;
    packet.adc3 = adc3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_adc_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_adc;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN);
#endif
}

/**
 * @brief Pack a adc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc1  adc1 value
 * @param adc2  adc2 value
 * @param adc3  adc3 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t adc1,uint16_t adc2,uint16_t adc3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_adc_LEN];
    _mav_put_uint16_t(buf, 0, adc1);
    _mav_put_uint16_t(buf, 2, adc2);
    _mav_put_uint16_t(buf, 4, adc3);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_adc_LEN);
#else
    mavlink_adc_t packet;
    packet.adc1 = adc1;
    packet.adc2 = adc2;
    packet.adc3 = adc3;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_adc_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_adc;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
}

/**
 * @brief Encode a adc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adc_t* adc)
{
    return mavlink_msg_adc_pack(system_id, component_id, msg, adc->adc1, adc->adc2, adc->adc3);
}

/**
 * @brief Encode a adc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adc_t* adc)
{
    return mavlink_msg_adc_pack_chan(system_id, component_id, chan, msg, adc->adc1, adc->adc2, adc->adc3);
}

/**
 * @brief Encode a adc struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param adc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adc_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_adc_t* adc)
{
    return mavlink_msg_adc_pack_status(system_id, component_id, _status, msg,  adc->adc1, adc->adc2, adc->adc3);
}

/**
 * @brief Send a adc message
 * @param chan MAVLink channel to send the message
 *
 * @param adc1  adc1 value
 * @param adc2  adc2 value
 * @param adc3  adc3 value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adc_send(mavlink_channel_t chan, uint16_t adc1, uint16_t adc2, uint16_t adc3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_adc_LEN];
    _mav_put_uint16_t(buf, 0, adc1);
    _mav_put_uint16_t(buf, 2, adc2);
    _mav_put_uint16_t(buf, 4, adc3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_adc, buf, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
#else
    mavlink_adc_t packet;
    packet.adc1 = adc1;
    packet.adc2 = adc2;
    packet.adc3 = adc3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_adc, (const char *)&packet, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
#endif
}

/**
 * @brief Send a adc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adc_send_struct(mavlink_channel_t chan, const mavlink_adc_t* adc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adc_send(chan, adc->adc1, adc->adc2, adc->adc3);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_adc, (const char *)adc, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
#endif
}

#if MAVLINK_MSG_ID_adc_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t adc1, uint16_t adc2, uint16_t adc3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, adc1);
    _mav_put_uint16_t(buf, 2, adc2);
    _mav_put_uint16_t(buf, 4, adc3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_adc, buf, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
#else
    mavlink_adc_t *packet = (mavlink_adc_t *)msgbuf;
    packet->adc1 = adc1;
    packet->adc2 = adc2;
    packet->adc3 = adc3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_adc, (const char *)packet, MAVLINK_MSG_ID_adc_MIN_LEN, MAVLINK_MSG_ID_adc_LEN, MAVLINK_MSG_ID_adc_CRC);
#endif
}
#endif

#endif

// MESSAGE adc UNPACKING


/**
 * @brief Get field adc1 from adc message
 *
 * @return  adc1 value
 */
static inline uint16_t mavlink_msg_adc_get_adc1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field adc2 from adc message
 *
 * @return  adc2 value
 */
static inline uint16_t mavlink_msg_adc_get_adc2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field adc3 from adc message
 *
 * @return  adc3 value
 */
static inline uint16_t mavlink_msg_adc_get_adc3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a adc message into a struct
 *
 * @param msg The message to decode
 * @param adc C-struct to decode the message contents into
 */
static inline void mavlink_msg_adc_decode(const mavlink_message_t* msg, mavlink_adc_t* adc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    adc->adc1 = mavlink_msg_adc_get_adc1(msg);
    adc->adc2 = mavlink_msg_adc_get_adc2(msg);
    adc->adc3 = mavlink_msg_adc_get_adc3(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_adc_LEN? msg->len : MAVLINK_MSG_ID_adc_LEN;
        memset(adc, 0, MAVLINK_MSG_ID_adc_LEN);
    memcpy(adc, _MAV_PAYLOAD(msg), len);
#endif
}
