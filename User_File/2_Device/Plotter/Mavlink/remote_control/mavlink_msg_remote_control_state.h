#pragma once
// MESSAGE REMOTE_CONTROL_STATE PACKING

#define MAVLINK_MSG_ID_REMOTE_CONTROL_STATE 150


typedef struct __mavlink_remote_control_state_t {
 uint32_t kfs; /*<  梅林KFS按键矩阵4x3，每元素2位。屏幕上报1=空/2=R1/3=R2/4=假，固件减1后打包为0/1/2/3，共24位有效（bit0~bit23）*/
 int16_t encoder_0; /*<  编码器0旋转位置值*/
 int16_t encoder_1; /*<  编码器1旋转位置值*/
 int16_t encoder_2; /*<  编码器2旋转位置值*/
 int16_t encoder_3; /*<  编码器3旋转位置值*/
 int16_t move_0; /*<  摇杆通道0移动量*/
 int16_t move_1; /*<  摇杆通道1移动量*/
 int16_t move_2; /*<  摇杆通道2移动量*/
 int16_t move_3; /*<  摇杆通道3移动量*/
 uint8_t mode; /*<  4路模式拨码开关状态（bit0~bit3 对应 mode[0]~mode[3]，1=高电平/未拨下，0=低电平/拨下）*/
 uint8_t act; /*<  6动作独立位掩码：bit0~2=动作1~3按下态(双稳态)，bit3~5=动作4~6触发态(瞬时)，bit6~7保留*/
 uint8_t kfs_put; /*<  R2九宫格按键矩阵2x3，每元素1位。屏幕上报1=不放/2=放，固件减1后打包为0/1，共6位有效（bit0~bit5）*/
} mavlink_remote_control_state_t;

#define MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN 23
#define MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN 23
#define MAVLINK_MSG_ID_150_LEN 23
#define MAVLINK_MSG_ID_150_MIN_LEN 23

#define MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC 150
#define MAVLINK_MSG_ID_150_CRC 150



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_REMOTE_CONTROL_STATE { \
    150, \
    "REMOTE_CONTROL_STATE", \
    12, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_remote_control_state_t, mode) }, \
         { "encoder_0", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_remote_control_state_t, encoder_0) }, \
         { "encoder_1", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_remote_control_state_t, encoder_1) }, \
         { "encoder_2", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_remote_control_state_t, encoder_2) }, \
         { "encoder_3", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_remote_control_state_t, encoder_3) }, \
         { "move_0", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_remote_control_state_t, move_0) }, \
         { "move_1", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_remote_control_state_t, move_1) }, \
         { "move_2", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_remote_control_state_t, move_2) }, \
         { "move_3", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_remote_control_state_t, move_3) }, \
         { "act", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_remote_control_state_t, act) }, \
         { "kfs", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_remote_control_state_t, kfs) }, \
         { "kfs_put", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_remote_control_state_t, kfs_put) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_REMOTE_CONTROL_STATE { \
    "REMOTE_CONTROL_STATE", \
    12, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_remote_control_state_t, mode) }, \
         { "encoder_0", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_remote_control_state_t, encoder_0) }, \
         { "encoder_1", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_remote_control_state_t, encoder_1) }, \
         { "encoder_2", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_remote_control_state_t, encoder_2) }, \
         { "encoder_3", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_remote_control_state_t, encoder_3) }, \
         { "move_0", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_remote_control_state_t, move_0) }, \
         { "move_1", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_remote_control_state_t, move_1) }, \
         { "move_2", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_remote_control_state_t, move_2) }, \
         { "move_3", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_remote_control_state_t, move_3) }, \
         { "act", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_remote_control_state_t, act) }, \
         { "kfs", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_remote_control_state_t, kfs) }, \
         { "kfs_put", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_remote_control_state_t, kfs_put) }, \
         } \
}
#endif

/**
 * @brief Pack a remote_control_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  4路模式拨码开关状态（bit0~bit3 对应 mode[0]~mode[3]，1=高电平/未拨下，0=低电平/拨下）
 * @param encoder_0  编码器0旋转位置值
 * @param encoder_1  编码器1旋转位置值
 * @param encoder_2  编码器2旋转位置值
 * @param encoder_3  编码器3旋转位置值
 * @param move_0  摇杆通道0移动量
 * @param move_1  摇杆通道1移动量
 * @param move_2  摇杆通道2移动量
 * @param move_3  摇杆通道3移动量
 * @param act  6动作独立位掩码：bit0~2=动作1~3按下态(双稳态)，bit3~5=动作4~6触发态(瞬时)，bit6~7保留
 * @param kfs  梅林KFS按键矩阵4x3，每元素2位。屏幕上报1=空/2=R1/3=R2/4=假，固件减1后打包为0/1/2/3，共24位有效（bit0~bit23）
 * @param kfs_put  R2九宫格按键矩阵2x3，每元素1位。屏幕上报1=不放/2=放，固件减1后打包为0/1，共6位有效（bit0~bit5）
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_remote_control_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mode, int16_t encoder_0, int16_t encoder_1, int16_t encoder_2, int16_t encoder_3, int16_t move_0, int16_t move_1, int16_t move_2, int16_t move_3, uint8_t act, uint32_t kfs, uint8_t kfs_put)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, kfs);
    _mav_put_int16_t(buf, 4, encoder_0);
    _mav_put_int16_t(buf, 6, encoder_1);
    _mav_put_int16_t(buf, 8, encoder_2);
    _mav_put_int16_t(buf, 10, encoder_3);
    _mav_put_int16_t(buf, 12, move_0);
    _mav_put_int16_t(buf, 14, move_1);
    _mav_put_int16_t(buf, 16, move_2);
    _mav_put_int16_t(buf, 18, move_3);
    _mav_put_uint8_t(buf, 20, mode);
    _mav_put_uint8_t(buf, 21, act);
    _mav_put_uint8_t(buf, 22, kfs_put);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
#else
    mavlink_remote_control_state_t packet;
    packet.kfs = kfs;
    packet.encoder_0 = encoder_0;
    packet.encoder_1 = encoder_1;
    packet.encoder_2 = encoder_2;
    packet.encoder_3 = encoder_3;
    packet.move_0 = move_0;
    packet.move_1 = move_1;
    packet.move_2 = move_2;
    packet.move_3 = move_3;
    packet.mode = mode;
    packet.act = act;
    packet.kfs_put = kfs_put;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REMOTE_CONTROL_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
}

/**
 * @brief Pack a remote_control_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  4路模式拨码开关状态（bit0~bit3 对应 mode[0]~mode[3]，1=高电平/未拨下，0=低电平/拨下）
 * @param encoder_0  编码器0旋转位置值
 * @param encoder_1  编码器1旋转位置值
 * @param encoder_2  编码器2旋转位置值
 * @param encoder_3  编码器3旋转位置值
 * @param move_0  摇杆通道0移动量
 * @param move_1  摇杆通道1移动量
 * @param move_2  摇杆通道2移动量
 * @param move_3  摇杆通道3移动量
 * @param act  6动作独立位掩码：bit0~2=动作1~3按下态(双稳态)，bit3~5=动作4~6触发态(瞬时)，bit6~7保留
 * @param kfs  梅林KFS按键矩阵4x3，每元素2位。屏幕上报1=空/2=R1/3=R2/4=假，固件减1后打包为0/1/2/3，共24位有效（bit0~bit23）
 * @param kfs_put  R2九宫格按键矩阵2x3，每元素1位。屏幕上报1=不放/2=放，固件减1后打包为0/1，共6位有效（bit0~bit5）
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_remote_control_state_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t mode, int16_t encoder_0, int16_t encoder_1, int16_t encoder_2, int16_t encoder_3, int16_t move_0, int16_t move_1, int16_t move_2, int16_t move_3, uint8_t act, uint32_t kfs, uint8_t kfs_put)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, kfs);
    _mav_put_int16_t(buf, 4, encoder_0);
    _mav_put_int16_t(buf, 6, encoder_1);
    _mav_put_int16_t(buf, 8, encoder_2);
    _mav_put_int16_t(buf, 10, encoder_3);
    _mav_put_int16_t(buf, 12, move_0);
    _mav_put_int16_t(buf, 14, move_1);
    _mav_put_int16_t(buf, 16, move_2);
    _mav_put_int16_t(buf, 18, move_3);
    _mav_put_uint8_t(buf, 20, mode);
    _mav_put_uint8_t(buf, 21, act);
    _mav_put_uint8_t(buf, 22, kfs_put);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
#else
    mavlink_remote_control_state_t packet;
    packet.kfs = kfs;
    packet.encoder_0 = encoder_0;
    packet.encoder_1 = encoder_1;
    packet.encoder_2 = encoder_2;
    packet.encoder_3 = encoder_3;
    packet.move_0 = move_0;
    packet.move_1 = move_1;
    packet.move_2 = move_2;
    packet.move_3 = move_3;
    packet.mode = mode;
    packet.act = act;
    packet.kfs_put = kfs_put;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REMOTE_CONTROL_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
#endif
}

/**
 * @brief Pack a remote_control_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  4路模式拨码开关状态（bit0~bit3 对应 mode[0]~mode[3]，1=高电平/未拨下，0=低电平/拨下）
 * @param encoder_0  编码器0旋转位置值
 * @param encoder_1  编码器1旋转位置值
 * @param encoder_2  编码器2旋转位置值
 * @param encoder_3  编码器3旋转位置值
 * @param move_0  摇杆通道0移动量
 * @param move_1  摇杆通道1移动量
 * @param move_2  摇杆通道2移动量
 * @param move_3  摇杆通道3移动量
 * @param act  6动作独立位掩码：bit0~2=动作1~3按下态(双稳态)，bit3~5=动作4~6触发态(瞬时)，bit6~7保留
 * @param kfs  梅林KFS按键矩阵4x3，每元素2位。屏幕上报1=空/2=R1/3=R2/4=假，固件减1后打包为0/1/2/3，共24位有效（bit0~bit23）
 * @param kfs_put  R2九宫格按键矩阵2x3，每元素1位。屏幕上报1=不放/2=放，固件减1后打包为0/1，共6位有效（bit0~bit5）
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_remote_control_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mode,int16_t encoder_0,int16_t encoder_1,int16_t encoder_2,int16_t encoder_3,int16_t move_0,int16_t move_1,int16_t move_2,int16_t move_3,uint8_t act,uint32_t kfs,uint8_t kfs_put)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, kfs);
    _mav_put_int16_t(buf, 4, encoder_0);
    _mav_put_int16_t(buf, 6, encoder_1);
    _mav_put_int16_t(buf, 8, encoder_2);
    _mav_put_int16_t(buf, 10, encoder_3);
    _mav_put_int16_t(buf, 12, move_0);
    _mav_put_int16_t(buf, 14, move_1);
    _mav_put_int16_t(buf, 16, move_2);
    _mav_put_int16_t(buf, 18, move_3);
    _mav_put_uint8_t(buf, 20, mode);
    _mav_put_uint8_t(buf, 21, act);
    _mav_put_uint8_t(buf, 22, kfs_put);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
#else
    mavlink_remote_control_state_t packet;
    packet.kfs = kfs;
    packet.encoder_0 = encoder_0;
    packet.encoder_1 = encoder_1;
    packet.encoder_2 = encoder_2;
    packet.encoder_3 = encoder_3;
    packet.move_0 = move_0;
    packet.move_1 = move_1;
    packet.move_2 = move_2;
    packet.move_3 = move_3;
    packet.mode = mode;
    packet.act = act;
    packet.kfs_put = kfs_put;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_REMOTE_CONTROL_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
}

/**
 * @brief Encode a remote_control_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param remote_control_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_remote_control_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_remote_control_state_t* remote_control_state)
{
    return mavlink_msg_remote_control_state_pack(system_id, component_id, msg, remote_control_state->mode, remote_control_state->encoder_0, remote_control_state->encoder_1, remote_control_state->encoder_2, remote_control_state->encoder_3, remote_control_state->move_0, remote_control_state->move_1, remote_control_state->move_2, remote_control_state->move_3, remote_control_state->act, remote_control_state->kfs, remote_control_state->kfs_put);
}

/**
 * @brief Encode a remote_control_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param remote_control_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_remote_control_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_remote_control_state_t* remote_control_state)
{
    return mavlink_msg_remote_control_state_pack_chan(system_id, component_id, chan, msg, remote_control_state->mode, remote_control_state->encoder_0, remote_control_state->encoder_1, remote_control_state->encoder_2, remote_control_state->encoder_3, remote_control_state->move_0, remote_control_state->move_1, remote_control_state->move_2, remote_control_state->move_3, remote_control_state->act, remote_control_state->kfs, remote_control_state->kfs_put);
}

/**
 * @brief Encode a remote_control_state struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param remote_control_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_remote_control_state_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_remote_control_state_t* remote_control_state)
{
    return mavlink_msg_remote_control_state_pack_status(system_id, component_id, _status, msg,  remote_control_state->mode, remote_control_state->encoder_0, remote_control_state->encoder_1, remote_control_state->encoder_2, remote_control_state->encoder_3, remote_control_state->move_0, remote_control_state->move_1, remote_control_state->move_2, remote_control_state->move_3, remote_control_state->act, remote_control_state->kfs, remote_control_state->kfs_put);
}

/**
 * @brief Send a remote_control_state message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  4路模式拨码开关状态（bit0~bit3 对应 mode[0]~mode[3]，1=高电平/未拨下，0=低电平/拨下）
 * @param encoder_0  编码器0旋转位置值
 * @param encoder_1  编码器1旋转位置值
 * @param encoder_2  编码器2旋转位置值
 * @param encoder_3  编码器3旋转位置值
 * @param move_0  摇杆通道0移动量
 * @param move_1  摇杆通道1移动量
 * @param move_2  摇杆通道2移动量
 * @param move_3  摇杆通道3移动量
 * @param act  6动作独立位掩码：bit0~2=动作1~3按下态(双稳态)，bit3~5=动作4~6触发态(瞬时)，bit6~7保留
 * @param kfs  梅林KFS按键矩阵4x3，每元素2位。屏幕上报1=空/2=R1/3=R2/4=假，固件减1后打包为0/1/2/3，共24位有效（bit0~bit23）
 * @param kfs_put  R2九宫格按键矩阵2x3，每元素1位。屏幕上报1=不放/2=放，固件减1后打包为0/1，共6位有效（bit0~bit5）
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_remote_control_state_send(mavlink_channel_t chan, uint8_t mode, int16_t encoder_0, int16_t encoder_1, int16_t encoder_2, int16_t encoder_3, int16_t move_0, int16_t move_1, int16_t move_2, int16_t move_3, uint8_t act, uint32_t kfs, uint8_t kfs_put)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN];
    _mav_put_uint32_t(buf, 0, kfs);
    _mav_put_int16_t(buf, 4, encoder_0);
    _mav_put_int16_t(buf, 6, encoder_1);
    _mav_put_int16_t(buf, 8, encoder_2);
    _mav_put_int16_t(buf, 10, encoder_3);
    _mav_put_int16_t(buf, 12, move_0);
    _mav_put_int16_t(buf, 14, move_1);
    _mav_put_int16_t(buf, 16, move_2);
    _mav_put_int16_t(buf, 18, move_3);
    _mav_put_uint8_t(buf, 20, mode);
    _mav_put_uint8_t(buf, 21, act);
    _mav_put_uint8_t(buf, 22, kfs_put);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE, buf, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
#else
    mavlink_remote_control_state_t packet;
    packet.kfs = kfs;
    packet.encoder_0 = encoder_0;
    packet.encoder_1 = encoder_1;
    packet.encoder_2 = encoder_2;
    packet.encoder_3 = encoder_3;
    packet.move_0 = move_0;
    packet.move_1 = move_1;
    packet.move_2 = move_2;
    packet.move_3 = move_3;
    packet.mode = mode;
    packet.act = act;
    packet.kfs_put = kfs_put;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE, (const char *)&packet, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
#endif
}

/**
 * @brief Send a remote_control_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_remote_control_state_send_struct(mavlink_channel_t chan, const mavlink_remote_control_state_t* remote_control_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_remote_control_state_send(chan, remote_control_state->mode, remote_control_state->encoder_0, remote_control_state->encoder_1, remote_control_state->encoder_2, remote_control_state->encoder_3, remote_control_state->move_0, remote_control_state->move_1, remote_control_state->move_2, remote_control_state->move_3, remote_control_state->act, remote_control_state->kfs, remote_control_state->kfs_put);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE, (const char *)remote_control_state, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_remote_control_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode, int16_t encoder_0, int16_t encoder_1, int16_t encoder_2, int16_t encoder_3, int16_t move_0, int16_t move_1, int16_t move_2, int16_t move_3, uint8_t act, uint32_t kfs, uint8_t kfs_put)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, kfs);
    _mav_put_int16_t(buf, 4, encoder_0);
    _mav_put_int16_t(buf, 6, encoder_1);
    _mav_put_int16_t(buf, 8, encoder_2);
    _mav_put_int16_t(buf, 10, encoder_3);
    _mav_put_int16_t(buf, 12, move_0);
    _mav_put_int16_t(buf, 14, move_1);
    _mav_put_int16_t(buf, 16, move_2);
    _mav_put_int16_t(buf, 18, move_3);
    _mav_put_uint8_t(buf, 20, mode);
    _mav_put_uint8_t(buf, 21, act);
    _mav_put_uint8_t(buf, 22, kfs_put);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE, buf, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
#else
    mavlink_remote_control_state_t *packet = (mavlink_remote_control_state_t *)msgbuf;
    packet->kfs = kfs;
    packet->encoder_0 = encoder_0;
    packet->encoder_1 = encoder_1;
    packet->encoder_2 = encoder_2;
    packet->encoder_3 = encoder_3;
    packet->move_0 = move_0;
    packet->move_1 = move_1;
    packet->move_2 = move_2;
    packet->move_3 = move_3;
    packet->mode = mode;
    packet->act = act;
    packet->kfs_put = kfs_put;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE, (const char *)packet, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE REMOTE_CONTROL_STATE UNPACKING


/**
 * @brief Get field mode from remote_control_state message
 *
 * @return  4路模式拨码开关状态（bit0~bit3 对应 mode[0]~mode[3]，1=高电平/未拨下，0=低电平/拨下）
 */
static inline uint8_t mavlink_msg_remote_control_state_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field encoder_0 from remote_control_state message
 *
 * @return  编码器0旋转位置值
 */
static inline int16_t mavlink_msg_remote_control_state_get_encoder_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field encoder_1 from remote_control_state message
 *
 * @return  编码器1旋转位置值
 */
static inline int16_t mavlink_msg_remote_control_state_get_encoder_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field encoder_2 from remote_control_state message
 *
 * @return  编码器2旋转位置值
 */
static inline int16_t mavlink_msg_remote_control_state_get_encoder_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field encoder_3 from remote_control_state message
 *
 * @return  编码器3旋转位置值
 */
static inline int16_t mavlink_msg_remote_control_state_get_encoder_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field move_0 from remote_control_state message
 *
 * @return  摇杆通道0移动量
 */
static inline int16_t mavlink_msg_remote_control_state_get_move_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field move_1 from remote_control_state message
 *
 * @return  摇杆通道1移动量
 */
static inline int16_t mavlink_msg_remote_control_state_get_move_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field move_2 from remote_control_state message
 *
 * @return  摇杆通道2移动量
 */
static inline int16_t mavlink_msg_remote_control_state_get_move_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field move_3 from remote_control_state message
 *
 * @return  摇杆通道3移动量
 */
static inline int16_t mavlink_msg_remote_control_state_get_move_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field act from remote_control_state message
 *
 * @return  6动作独立位掩码：bit0~2=动作1~3按下态(双稳态)，bit3~5=动作4~6触发态(瞬时)，bit6~7保留
 */
static inline uint8_t mavlink_msg_remote_control_state_get_act(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field kfs from remote_control_state message
 *
 * @return  梅林KFS按键矩阵4x3，每元素2位。屏幕上报1=空/2=R1/3=R2/4=假，固件减1后打包为0/1/2/3，共24位有效（bit0~bit23）
 */
static inline uint32_t mavlink_msg_remote_control_state_get_kfs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field kfs_put from remote_control_state message
 *
 * @return  R2九宫格按键矩阵2x3，每元素1位。屏幕上报1=不放/2=放，固件减1后打包为0/1，共6位有效（bit0~bit5）
 */
static inline uint8_t mavlink_msg_remote_control_state_get_kfs_put(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Decode a remote_control_state message into a struct
 *
 * @param msg The message to decode
 * @param remote_control_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_remote_control_state_decode(const mavlink_message_t* msg, mavlink_remote_control_state_t* remote_control_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    remote_control_state->kfs = mavlink_msg_remote_control_state_get_kfs(msg);
    remote_control_state->encoder_0 = mavlink_msg_remote_control_state_get_encoder_0(msg);
    remote_control_state->encoder_1 = mavlink_msg_remote_control_state_get_encoder_1(msg);
    remote_control_state->encoder_2 = mavlink_msg_remote_control_state_get_encoder_2(msg);
    remote_control_state->encoder_3 = mavlink_msg_remote_control_state_get_encoder_3(msg);
    remote_control_state->move_0 = mavlink_msg_remote_control_state_get_move_0(msg);
    remote_control_state->move_1 = mavlink_msg_remote_control_state_get_move_1(msg);
    remote_control_state->move_2 = mavlink_msg_remote_control_state_get_move_2(msg);
    remote_control_state->move_3 = mavlink_msg_remote_control_state_get_move_3(msg);
    remote_control_state->mode = mavlink_msg_remote_control_state_get_mode(msg);
    remote_control_state->act = mavlink_msg_remote_control_state_get_act(msg);
    remote_control_state->kfs_put = mavlink_msg_remote_control_state_get_kfs_put(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN? msg->len : MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN;
        memset(remote_control_state, 0, MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_LEN);
    memcpy(remote_control_state, _MAV_PAYLOAD(msg), len);
#endif
}
