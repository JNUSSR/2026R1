/** @file
 *    @brief MAVLink comm protocol testsuite generated from remote_control_v2.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef REMOTE_CONTROL_V2_TESTSUITE_H
#define REMOTE_CONTROL_V2_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_remote_control_v2(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_remote_control_v2(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_kfs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_KFS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_kfs_t packet_in = {
        963497464
    };
    mavlink_kfs_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.kfs = packet_in.kfs;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_KFS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_KFS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_kfs_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_kfs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_kfs_pack(system_id, component_id, &msg , packet1.kfs );
    mavlink_msg_kfs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_kfs_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.kfs );
    mavlink_msg_kfs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_kfs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_kfs_send(MAVLINK_COMM_1 , packet1.kfs );
    mavlink_msg_kfs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("KFS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_KFS) != NULL);
#endif
}

static void mavlink_test_act(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_act_t packet_in = {
        5
    };
    mavlink_act_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.act = packet_in.act;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_act_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_act_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_act_pack(system_id, component_id, &msg , packet1.act );
    mavlink_msg_act_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_act_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.act );
    mavlink_msg_act_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_act_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_act_send(MAVLINK_COMM_1 , packet1.act );
    mavlink_msg_act_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACT) != NULL);
#endif
}

static void mavlink_test_adc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_adc >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adc_t packet_in = {
        17235,17339,17443
    };
    mavlink_adc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.adc1 = packet_in.adc1;
        packet1.adc2 = packet_in.adc2;
        packet1.adc3 = packet_in.adc3;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_adc_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_adc_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_pack(system_id, component_id, &msg , packet1.adc1 , packet1.adc2 , packet1.adc3 );
    mavlink_msg_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.adc1 , packet1.adc2 , packet1.adc3 );
    mavlink_msg_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_send(MAVLINK_COMM_1 , packet1.adc1 , packet1.adc2 , packet1.adc3 );
    mavlink_msg_adc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("adc") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_adc) != NULL);
#endif
}

static void mavlink_test_remote_control_v2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_kfs(system_id, component_id, last_msg);
    mavlink_test_act(system_id, component_id, last_msg);
    mavlink_test_adc(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // REMOTE_CONTROL_V2_TESTSUITE_H
