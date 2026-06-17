/** @file
 *  @brief MAVLink comm protocol generated from remote_control.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_REMOTE_CONTROL_H
#define MAVLINK_REMOTE_CONTROL_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_REMOTE_CONTROL.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_REMOTE_CONTROL_XML_HASH 9170783198409209361

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{150, 150, 23, 23, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_REMOTE_CONTROL

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_remote_control_state.h"

// base include



#if MAVLINK_REMOTE_CONTROL_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_REMOTE_CONTROL_STATE}
# define MAVLINK_MESSAGE_NAMES {{ "REMOTE_CONTROL_STATE", 150 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_REMOTE_CONTROL_H
