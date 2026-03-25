// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2026
//

#include "hostmux.h"
#include "radio.h"
#include "crc.h"
#include "tdm.h"

#define HOSTMUX_FRAME_TYPE_TO_NODE   1
#define HOSTMUX_FRAME_TYPE_FROM_NODE 2
#define HOSTMUX_FRAME_TYPE_AT_CMD    3
#define HOSTMUX_FRAME_TYPE_AT_REPLY  4

#define HOSTMUX_FIELD_TYPE      0
#define HOSTMUX_FIELD_NODE_LOW  1
#define HOSTMUX_FIELD_NODE_HIGH 2
#define HOSTMUX_FIELD_LENGTH    3
#define HOSTMUX_FIELD_PAYLOAD   4

#define HOSTMUX_FRAME_OVERHEAD  6
#define HOSTMUX_MAX_PAYLOAD     ((uint8_t)(MAX_PACKET_LENGTH - 8))
#define HOSTMUX_FRAME_MAX       (HOSTMUX_MAX_PAYLOAD + HOSTMUX_FRAME_OVERHEAD)
#define HOSTMUX_ENCODED_MAX     (HOSTMUX_FRAME_MAX + 2)

static __bit hostmux_mode_enabled;
static __bit hostmux_pending_valid;
static __bit hostmux_rx_overflow;
static __pdata uint8_t hostmux_rx_encoded_len;
static __pdata uint8_t hostmux_pending_len;
static __pdata uint8_t hostmux_max_payload = HOSTMUX_MAX_PAYLOAD;
static __pdata uint16_t hostmux_pending_destination;
static __xdata uint8_t hostmux_pending_payload[HOSTMUX_MAX_PAYLOAD];
static __xdata uint8_t hostmux_encoded_buf[HOSTMUX_ENCODED_MAX];
static __xdata uint8_t hostmux_frame_buf[HOSTMUX_FRAME_MAX];

static uint8_t
hostmux_cobs_encode(__pdata uint8_t length, __xdata uint8_t *src, __xdata uint8_t *dst)
{
	__xdata uint8_t *code_ptr = dst++;
	__pdata uint8_t code = 1;
	__xdata uint8_t *start = code_ptr;

	while (length--) {
		register uint8_t c = *src++;
		if (c == 0) {
			*code_ptr = code;
			code_ptr = dst++;
			code = 1;
		} else {
			*dst++ = c;
			code++;
			if (code == 0xFF) {
				*code_ptr = code;
				code_ptr = dst++;
				code = 1;
			}
		}
	}

	*code_ptr = code;
	return (uint8_t)(dst - start);
}

static bool
hostmux_cobs_decode(__pdata uint8_t length, __xdata uint8_t *src, __xdata uint8_t *dst, __pdata uint8_t *decoded_length)
{
	__pdata uint8_t out_len = 0;

	while (length > 0) {
		__pdata uint8_t code = *src++;
		__pdata uint8_t i;

		length--;
		if (code == 0 || code > (uint8_t)(length + 1)) {
			return false;
		}

		for (i = 1; i < code; i++) {
			dst[out_len++] = *src++;
			length--;
		}

		if (code != 0xFF && length > 0) {
			dst[out_len++] = 0;
		}
	}

	*decoded_length = out_len;
	return true;
}

static void
hostmux_reset_parser(void)
{
	hostmux_rx_encoded_len = 0;
	hostmux_rx_overflow = false;
	hostmux_pending_valid = false;
	hostmux_pending_len = 0;
	hostmux_pending_destination = 0;
	at_mode_active = false;
}

static bool
hostmux_send_frame(__pdata uint8_t frame_type, __pdata uint16_t node, __xdata uint8_t * __pdata buf, __pdata uint8_t len)
{
	__pdata uint16_t frame_crc;
	__pdata uint8_t frame_len;
	__pdata uint8_t encoded_len;

	if (len > hostmux_max_payload) {
		return false;
	}

	hostmux_frame_buf[HOSTMUX_FIELD_TYPE] = frame_type;
	hostmux_frame_buf[HOSTMUX_FIELD_NODE_LOW] = (uint8_t)(node & 0xFF);
	hostmux_frame_buf[HOSTMUX_FIELD_NODE_HIGH] = (uint8_t)(node >> 8);
	hostmux_frame_buf[HOSTMUX_FIELD_LENGTH] = len;
	if (len != 0) {
		memcpy(&hostmux_frame_buf[HOSTMUX_FIELD_PAYLOAD], buf, len);
	}

	frame_len = HOSTMUX_FIELD_PAYLOAD + len;
	frame_crc = crc16(frame_len, hostmux_frame_buf);
	hostmux_frame_buf[frame_len++] = (uint8_t)(frame_crc >> 8);
	hostmux_frame_buf[frame_len++] = (uint8_t)(frame_crc & 0xFF);

	encoded_len = hostmux_cobs_encode(frame_len, hostmux_frame_buf, hostmux_encoded_buf);
	hostmux_encoded_buf[encoded_len++] = 0;
	if (serial_write_space() < encoded_len) {
		return false;
	}
	serial_write_buf(hostmux_encoded_buf, encoded_len);
	return true;
}

static void
hostmux_run_local_at(__pdata uint8_t payload_len)
{
	__pdata uint8_t cmd_len = payload_len;
	__pdata uint8_t reply_len;
	__pdata uint8_t i;

	while (cmd_len > 0) {
		register uint8_t c = hostmux_frame_buf[HOSTMUX_FIELD_PAYLOAD + cmd_len - 1];
		if (c != '\r' && c != '\n') {
			break;
		}
		cmd_len--;
	}
	if (cmd_len == 0) {
		return;
	}
	if (cmd_len > AT_CMD_MAXLEN) {
		static __xdata uint8_t at_error[] = "ERROR\r\n";
		hostmux_send_frame(HOSTMUX_FRAME_TYPE_AT_REPLY, BASE_NODEID, at_error, sizeof(at_error) - 1);
		return;
	}

	for (i = 0; i < cmd_len; i++) {
		at_cmd[i] = toupper(hostmux_frame_buf[HOSTMUX_FIELD_PAYLOAD + i]);
	}
	at_cmd[cmd_len] = '\0';
	at_cmd_len = cmd_len;
	at_cmd_ready = true;

	/* ATI5 lists all parameters — its total output (~600 bytes across 20 frames)
	 * exceeds the single hostmux_max_payload capture buffer (244 bytes max) AND
	 * can overflow the 512-byte serial TX buffer if sent in a tight loop.
	 * Fix: send one COBS AT_REPLY frame per parameter.  Before each send, spin
	 * briefly so the UART TX ISR can drain the buffer — the spin is bounded to
	 * ~8ms max so TDM timing impact is negligible (only frames 18-20 ever wait).
	 */
	if (cmd_len == 4 && at_cmd[0] == 'A' && at_cmd[1] == 'T' &&
	    at_cmd[2] == 'I' && at_cmd[3] == '5') {
		for (i = 0; i < PARAM_MAX; i++) {
			__pdata uint16_t wait;
			printf_start_capture(hostmux_encoded_buf, hostmux_max_payload);
			param_print(i);
			reply_len = printf_end_capture();
			if (reply_len > 0) {
				/* Spin until the TX buffer has room for this frame.
				 * UART ISR fires while we spin (EA=1) and drains bytes.
				 * At 57600 baud a 30-byte frame drains in ~5ms; 8000
				 * iterations at ~1us each gives ~8ms — enough headroom. */
				for (wait = 8000;
				     wait > 0 && serial_write_space() < (uint16_t)(reply_len + 4);
				     wait--)
					;
				hostmux_send_frame(HOSTMUX_FRAME_TYPE_AT_REPLY, BASE_NODEID,
				                   hostmux_encoded_buf, reply_len);
			}
		}
		at_cmd_ready = false;
		return;
	}

	printf_start_capture(hostmux_encoded_buf, hostmux_max_payload);
	at_command();
	reply_len = printf_end_capture();
	at_cmd_ready = false;

	if (reply_len > 0) {
		hostmux_send_frame(HOSTMUX_FRAME_TYPE_AT_REPLY, BASE_NODEID, hostmux_encoded_buf, reply_len);
	}
}

static bool
hostmux_accept_frame(__pdata uint8_t encoded_len)
{
	__pdata uint8_t decoded_len;
	__pdata uint8_t payload_len;
	__pdata uint8_t frame_type;
	__pdata uint16_t destination;
	__pdata uint16_t frame_crc;
	__pdata uint16_t calc_crc;

	if (!hostmux_cobs_decode(encoded_len, hostmux_encoded_buf, hostmux_frame_buf, &decoded_len)) {
		return false;
	}
	if (decoded_len < HOSTMUX_FRAME_OVERHEAD) {
		return false;
	}
	payload_len = hostmux_frame_buf[HOSTMUX_FIELD_LENGTH];
	if (payload_len > hostmux_max_payload) {
		return false;
	}
	if (decoded_len != (uint8_t)(HOSTMUX_FIELD_PAYLOAD + payload_len + 2)) {
		return false;
	}

	frame_crc = ((uint16_t)hostmux_frame_buf[decoded_len-2] << 8) | hostmux_frame_buf[decoded_len-1];
	calc_crc = crc16(decoded_len-2, hostmux_frame_buf);
	if (frame_crc != calc_crc) {
		return false;
	}

	frame_type = hostmux_frame_buf[HOSTMUX_FIELD_TYPE];
	destination = hostmux_frame_buf[HOSTMUX_FIELD_NODE_LOW] | ((uint16_t)hostmux_frame_buf[HOSTMUX_FIELD_NODE_HIGH] << 8);

	if (frame_type == HOSTMUX_FRAME_TYPE_TO_NODE) {
		if (destination == BASE_NODEID) {
			return false;
		}
		memcpy(hostmux_pending_payload, &hostmux_frame_buf[HOSTMUX_FIELD_PAYLOAD], payload_len);
		hostmux_pending_destination = destination;
		hostmux_pending_len = payload_len;
		hostmux_pending_valid = true;
		return true;
	}

	if (frame_type == HOSTMUX_FRAME_TYPE_AT_CMD) {
		hostmux_run_local_at(payload_len);
		return true;
	}

	return false;
}

void
hostmux_poll(void)
{
	if (!hostmux_enabled()) {
		return;
	}
	while (!hostmux_pending_valid && serial_read_available() > 0) {
		register uint8_t c = serial_read();
		if (c == 0) {
			if (!hostmux_rx_overflow && hostmux_rx_encoded_len > 0) {
				hostmux_accept_frame(hostmux_rx_encoded_len);
			}
			hostmux_rx_encoded_len = 0;
			hostmux_rx_overflow = false;
			continue;
		}

		if (hostmux_rx_overflow) {
			continue;
		}
		if (hostmux_rx_encoded_len >= sizeof(hostmux_encoded_buf)) {
			hostmux_rx_overflow = true;
			hostmux_rx_encoded_len = 0;
			continue;
		}
		hostmux_encoded_buf[hostmux_rx_encoded_len++] = c;
	}
}

void
hostmux_set_mode(__pdata uint8_t mode)
{
	hostmux_mode_enabled = (mode == HOSTMUX_MODE_COBS);
	hostmux_reset_parser();
}

void
hostmux_set_max_payload(__pdata uint8_t max_payload)
{
	if (max_payload > HOSTMUX_MAX_PAYLOAD) {
		max_payload = HOSTMUX_MAX_PAYLOAD;
	}
	hostmux_max_payload = max_payload;
}

bool
hostmux_enabled(void)
{
	return hostmux_mode_enabled && (nodeId == BASE_NODEID);
}

void
hostmux_deliver(__pdata uint16_t source_node, __xdata uint8_t * __pdata buf, __pdata uint8_t len)
{
	if (!hostmux_enabled()) {
		serial_write_buf(buf, len);
		return;
	}
	if (!hostmux_send_frame(HOSTMUX_FRAME_TYPE_FROM_NODE, source_node, buf, len)) {
		if (errors.serial_tx_overflow != 0xFFFF) {
			errors.serial_tx_overflow++;
		}
	}
}

uint8_t
hostmux_get_next(__pdata uint8_t max_xmit, __xdata uint8_t * __pdata buf, __pdata uint16_t *destination)
{
	if (!hostmux_enabled()) {
		return 0;
	}
	if (!hostmux_pending_valid) {
		hostmux_poll();
	}
	if (!hostmux_pending_valid) {
		return 0;
	}
	if (hostmux_pending_len > max_xmit) {
		return 0;
	}

	memcpy(buf, hostmux_pending_payload, hostmux_pending_len);
	*destination = hostmux_pending_destination;
	hostmux_pending_valid = false;
	return hostmux_pending_len;
}
