/** @file
 *    @brief MAVLink comm protocol testsuite generated from remote_control.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef REMOTE_CONTROL_TESTSUITE_H
#define REMOTE_CONTROL_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_remote_control(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_remote_control(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_remote_control_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REMOTE_CONTROL_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_remote_control_state_t packet_in = {
        963497464,17443,17547,17651,17755,17859,17963,18067,18171,65,132,199
    };
    mavlink_remote_control_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.kfs = packet_in.kfs;
        packet1.encoder_0 = packet_in.encoder_0;
        packet1.encoder_1 = packet_in.encoder_1;
        packet1.encoder_2 = packet_in.encoder_2;
        packet1.encoder_3 = packet_in.encoder_3;
        packet1.move_0 = packet_in.move_0;
        packet1.move_1 = packet_in.move_1;
        packet1.move_2 = packet_in.move_2;
        packet1.move_3 = packet_in.move_3;
        packet1.mode = packet_in.mode;
        packet1.act = packet_in.act;
        packet1.kfs_put = packet_in.kfs_put;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REMOTE_CONTROL_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_control_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_remote_control_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_control_state_pack(system_id, component_id, &msg , packet1.mode , packet1.encoder_0 , packet1.encoder_1 , packet1.encoder_2 , packet1.encoder_3 , packet1.move_0 , packet1.move_1 , packet1.move_2 , packet1.move_3 , packet1.act , packet1.kfs , packet1.kfs_put );
    mavlink_msg_remote_control_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_control_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mode , packet1.encoder_0 , packet1.encoder_1 , packet1.encoder_2 , packet1.encoder_3 , packet1.move_0 , packet1.move_1 , packet1.move_2 , packet1.move_3 , packet1.act , packet1.kfs , packet1.kfs_put );
    mavlink_msg_remote_control_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_remote_control_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_control_state_send(MAVLINK_COMM_1 , packet1.mode , packet1.encoder_0 , packet1.encoder_1 , packet1.encoder_2 , packet1.encoder_3 , packet1.move_0 , packet1.move_1 , packet1.move_2 , packet1.move_3 , packet1.act , packet1.kfs , packet1.kfs_put );
    mavlink_msg_remote_control_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REMOTE_CONTROL_STATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REMOTE_CONTROL_STATE) != NULL);
#endif
}

static void mavlink_test_remote_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_remote_control_state(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // REMOTE_CONTROL_TESTSUITE_H
