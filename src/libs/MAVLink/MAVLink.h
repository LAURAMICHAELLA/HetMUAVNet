#ifndef MAVLINK_MAVLINK_H
#define MAVLINK_MAVLINK_H

#include <stddef.h>

#define MAVLINK_ALIGNED_FIELDS 0
#define MAVLINK_USE_MESSAGE_INFO
#include "mavlink/v2.0/ardupilotmega/mavlink.h"

namespace MAVLink
{

/* Dump message contents to stdout (for debug/tracing purposes) */
void print_message(const mavlink_message_t *msg);

}

#endif // MAVLINK_MAVLINK_H
