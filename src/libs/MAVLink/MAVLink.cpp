#include "MAVLink/MAVLink.h"

#include <stddef.h>
#include <stdio.h>

namespace MAVLink
{

/* The following functions were borrowed from the mavlink project:
 *  mavlink/pymavlink/generator/C/test/posix/testmav.c
 */

static void print_one_field(const mavlink_message_t *msg, const mavlink_field_info_t *f, int idx)
{
#define PRINT_FORMAT(f, def) (f->print_format?f->print_format:def)
	switch (f->type) {
	case MAVLINK_TYPE_CHAR:
		printf(PRINT_FORMAT(f, "%c"), _MAV_RETURN_char(msg, f->wire_offset+idx*1));
		break;
	case MAVLINK_TYPE_UINT8_T:
		printf(PRINT_FORMAT(f, "%u"), _MAV_RETURN_uint8_t(msg, f->wire_offset+idx*1));
		break;
	case MAVLINK_TYPE_INT8_T:
		printf(PRINT_FORMAT(f, "%d"), _MAV_RETURN_int8_t(msg, f->wire_offset+idx*1));
		break;
	case MAVLINK_TYPE_UINT16_T:
		printf(PRINT_FORMAT(f, "%u"), _MAV_RETURN_uint16_t(msg, f->wire_offset+idx*2));
		break;
	case MAVLINK_TYPE_INT16_T:
		printf(PRINT_FORMAT(f, "%d"), _MAV_RETURN_int16_t(msg, f->wire_offset+idx*2));
		break;
	case MAVLINK_TYPE_UINT32_T:
		printf(PRINT_FORMAT(f, "%lu"), (unsigned long)_MAV_RETURN_uint32_t(msg, f->wire_offset+idx*4));
		break;
	case MAVLINK_TYPE_INT32_T:
		printf(PRINT_FORMAT(f, "%ld"), (long)_MAV_RETURN_int32_t(msg, f->wire_offset+idx*4));
		break;
	case MAVLINK_TYPE_UINT64_T:
		printf(PRINT_FORMAT(f, "%llu"), (unsigned long long)_MAV_RETURN_uint64_t(msg, f->wire_offset+idx*8));
		break;
	case MAVLINK_TYPE_INT64_T:
		printf(PRINT_FORMAT(f, "%lld"), (long long)_MAV_RETURN_int64_t(msg, f->wire_offset+idx*8));
		break;
	case MAVLINK_TYPE_FLOAT:
		printf(PRINT_FORMAT(f, "%f"), (double)_MAV_RETURN_float(msg, f->wire_offset+idx*4));
		break;
	case MAVLINK_TYPE_DOUBLE:
		printf(PRINT_FORMAT(f, "%f"), _MAV_RETURN_double(msg, f->wire_offset+idx*8));
		break;
	}
}

static void print_field(const mavlink_message_t *msg, const mavlink_field_info_t *f)
{
	printf("%s: ", f->name);
	if (f->array_length == 0) {
		print_one_field(msg, f, 0);
		printf(" ");
	} else {
		unsigned i;
		/* print an array */
		if (f->type == MAVLINK_TYPE_CHAR) {
			printf("'%.*s'", f->array_length,
			       f->wire_offset+(const char *)_MAV_PAYLOAD(msg));
			
		} else {
			printf("[ ");
			for (i=0; i<f->array_length; i++) {
				print_one_field(msg, f, i);
				if (i < f->array_length) {
					printf(", ");
				}
			}
			printf("]");
		}
	}
	printf(" ");
}

void print_message(const mavlink_message_t *msg)
{
	const mavlink_message_info_t *m = mavlink_get_message_info(msg);
	if (m == NULL) {
		printf("ERROR: no message info for %u\n", msg->msgid);
		//error_count++;
		return;
	}

	printf("%s { ", m->name);

	const mavlink_field_info_t *f = m->fields;
	for (unsigned i = 0; i < m->num_fields; i++) {
		print_field(msg, &f[i]);
	}
	printf("}\n");
}

}
