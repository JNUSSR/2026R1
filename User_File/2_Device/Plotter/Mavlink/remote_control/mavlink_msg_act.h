#pragma once
// MESSAGE ACT PACKING

#define MAVLINK_MSG_ID_ACT 201


typedef struct __mavlink_act_t {
 uint8_t act; /*<  act value*/
} mavlink_act_t;

#define MAVLINK_MSG_ID_ACT_LEN 1
#define MAVLINK_MSG_ID_ACT_MIN_LEN 1
#define MAVLINK_MSG_ID_201_LEN 1
#define MAVLINK_MSG_ID_201_MIN_LEN 1

#define MAVLINK_MSG_ID_ACT_CRC 214
#define MAVLINK_MSG_ID_201_CRC 214



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACT { \
    201, \
    "ACT", \
    1, \
    {  { "act", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_act_t, act) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACT { \
    "ACT", \
    1, \
    {  { "act", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_act_t, act) }, \
         } \
}
#endif

/**
 * @brief Pack a act message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param act  act value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_act_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACT_LEN];
    _mav_put_uint8_t(buf, 0, act);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACT_LEN);
#else
    mavlink_act_t packet;
    packet.act = act;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
}

/**
 * @brief Pack a act message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param act  act value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_act_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACT_LEN];
    _mav_put_uint8_t(buf, 0, act);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACT_LEN);
#else
    mavlink_act_t packet;
    packet.act = act;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN);
#endif
}

/**
 * @brief Pack a act message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param act  act value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_act_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACT_LEN];
    _mav_put_uint8_t(buf, 0, act);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACT_LEN);
#else
    mavlink_act_t packet;
    packet.act = act;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
}

/**
 * @brief Encode a act struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param act C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_act_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_act_t* act)
{
    return mavlink_msg_act_pack(system_id, component_id, msg, act->act);
}

/**
 * @brief Encode a act struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param act C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_act_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_act_t* act)
{
    return mavlink_msg_act_pack_chan(system_id, component_id, chan, msg, act->act);
}

/**
 * @brief Encode a act struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param act C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_act_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_act_t* act)
{
    return mavlink_msg_act_pack_status(system_id, component_id, _status, msg,  act->act);
}

/**
 * @brief Send a act message
 * @param chan MAVLink channel to send the message
 *
 * @param act  act value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_act_send(mavlink_channel_t chan, uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACT_LEN];
    _mav_put_uint8_t(buf, 0, act);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACT, buf, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
#else
    mavlink_act_t packet;
    packet.act = act;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACT, (const char *)&packet, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
#endif
}

/**
 * @brief Send a act message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_act_send_struct(mavlink_channel_t chan, const mavlink_act_t* act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_act_send(chan, act->act);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACT, (const char *)act, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_act_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, act);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACT, buf, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
#else
    mavlink_act_t *packet = (mavlink_act_t *)msgbuf;
    packet->act = act;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACT, (const char *)packet, MAVLINK_MSG_ID_ACT_MIN_LEN, MAVLINK_MSG_ID_ACT_LEN, MAVLINK_MSG_ID_ACT_CRC);
#endif
}
#endif

#endif

// MESSAGE ACT UNPACKING


/**
 * @brief Get field act from act message
 *
 * @return  act value
 */
static inline uint8_t mavlink_msg_act_get_act(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a act message into a struct
 *
 * @param msg The message to decode
 * @param act C-struct to decode the message contents into
 */
static inline void mavlink_msg_act_decode(const mavlink_message_t* msg, mavlink_act_t* act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    act->act = mavlink_msg_act_get_act(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACT_LEN? msg->len : MAVLINK_MSG_ID_ACT_LEN;
        memset(act, 0, MAVLINK_MSG_ID_ACT_LEN);
    memcpy(act, _MAV_PAYLOAD(msg), len);
#endif
}
