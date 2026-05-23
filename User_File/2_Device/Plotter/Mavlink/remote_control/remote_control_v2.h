/** @file
 *  @brief MAVLink comm protocol generated from remote_control_v2.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_REMOTE_CONTROL_V2_H
#define MAVLINK_REMOTE_CONTROL_V2_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_REMOTE_CONTROL_V2.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_REMOTE_CONTROL_V2_XML_HASH 7674162156997674291

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{200, 2, 4, 4, 0, 0, 0}, {201, 214, 1, 1, 0, 0, 0}, {202, 135, 6, 6, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_REMOTE_CONTROL_V2

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 1
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_kfs.h"
#include "./mavlink_msg_act.h"
#include "./mavlink_msg_adc.h"

// base include



#if MAVLINK_REMOTE_CONTROL_V2_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_KFS, MAVLINK_MESSAGE_INFO_ACT, MAVLINK_MESSAGE_INFO_adc}
# define MAVLINK_MESSAGE_NAMES {{ "ACT", 201 }, { "KFS", 200 }, { "adc", 202 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_REMOTE_CONTROL_V2_H
