#pragma once
// MESSAGE KFS PACKING

#define MAVLINK_MSG_ID_KFS 200


typedef struct __mavlink_kfs_t {
 uint32_t kfs; /*<  kfs value*/
} mavlink_kfs_t;

#define MAVLINK_MSG_ID_KFS_LEN 4
#define MAVLINK_MSG_ID_KFS_MIN_LEN 4
#define MAVLINK_MSG_ID_200_LEN 4
#define MAVLINK_MSG_ID_200_MIN_LEN 4

#define MAVLINK_MSG_ID_KFS_CRC 2
#define MAVLINK_MSG_ID_200_CRC 2



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_KFS { \
    200, \
    "KFS", \
    1, \
    {  { "kfs", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_kfs_t, kfs) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_KFS { \
    "KFS", \
    1, \
    {  { "kfs", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_kfs_t, kfs) }, \
         } \
}
#endif

/**
 * @brief Pack a kfs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param kfs  kfs value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kfs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t kfs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFS_LEN];
    _mav_put_uint32_t(buf, 0, kfs);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KFS_LEN);
#else
    mavlink_kfs_t packet;
    packet.kfs = kfs;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KFS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KFS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
}

/**
 * @brief Pack a kfs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param kfs  kfs value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kfs_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t kfs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFS_LEN];
    _mav_put_uint32_t(buf, 0, kfs);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KFS_LEN);
#else
    mavlink_kfs_t packet;
    packet.kfs = kfs;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KFS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KFS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN);
#endif
}

/**
 * @brief Pack a kfs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kfs  kfs value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kfs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t kfs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFS_LEN];
    _mav_put_uint32_t(buf, 0, kfs);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KFS_LEN);
#else
    mavlink_kfs_t packet;
    packet.kfs = kfs;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KFS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KFS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
}

/**
 * @brief Encode a kfs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param kfs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kfs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_kfs_t* kfs)
{
    return mavlink_msg_kfs_pack(system_id, component_id, msg, kfs->kfs);
}

/**
 * @brief Encode a kfs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kfs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kfs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_kfs_t* kfs)
{
    return mavlink_msg_kfs_pack_chan(system_id, component_id, chan, msg, kfs->kfs);
}

/**
 * @brief Encode a kfs struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param kfs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kfs_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_kfs_t* kfs)
{
    return mavlink_msg_kfs_pack_status(system_id, component_id, _status, msg,  kfs->kfs);
}

/**
 * @brief Send a kfs message
 * @param chan MAVLink channel to send the message
 *
 * @param kfs  kfs value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kfs_send(mavlink_channel_t chan, uint32_t kfs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KFS_LEN];
    _mav_put_uint32_t(buf, 0, kfs);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFS, buf, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
#else
    mavlink_kfs_t packet;
    packet.kfs = kfs;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFS, (const char *)&packet, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
#endif
}

/**
 * @brief Send a kfs message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_kfs_send_struct(mavlink_channel_t chan, const mavlink_kfs_t* kfs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_kfs_send(chan, kfs->kfs);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFS, (const char *)kfs, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
#endif
}

#if MAVLINK_MSG_ID_KFS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kfs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t kfs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, kfs);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFS, buf, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
#else
    mavlink_kfs_t *packet = (mavlink_kfs_t *)msgbuf;
    packet->kfs = kfs;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KFS, (const char *)packet, MAVLINK_MSG_ID_KFS_MIN_LEN, MAVLINK_MSG_ID_KFS_LEN, MAVLINK_MSG_ID_KFS_CRC);
#endif
}
#endif

#endif

// MESSAGE KFS UNPACKING


/**
 * @brief Get field kfs from kfs message
 *
 * @return  kfs value
 */
static inline uint32_t mavlink_msg_kfs_get_kfs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a kfs message into a struct
 *
 * @param msg The message to decode
 * @param kfs C-struct to decode the message contents into
 */
static inline void mavlink_msg_kfs_decode(const mavlink_message_t* msg, mavlink_kfs_t* kfs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    kfs->kfs = mavlink_msg_kfs_get_kfs(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_KFS_LEN? msg->len : MAVLINK_MSG_ID_KFS_LEN;
        memset(kfs, 0, MAVLINK_MSG_ID_KFS_LEN);
    memcpy(kfs, _MAV_PAYLOAD(msg), len);
#endif
}
