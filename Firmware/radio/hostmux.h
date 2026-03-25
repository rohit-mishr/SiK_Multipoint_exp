// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2026
//

#ifndef _HOSTMUX_H_
#define _HOSTMUX_H_

#include "radio.h"

#define HOSTMUX_MODE_DISABLED 0
#define HOSTMUX_MODE_COBS     1

extern void hostmux_set_mode(__pdata uint8_t mode);
extern void hostmux_set_max_payload(__pdata uint8_t max_payload);
extern bool hostmux_enabled(void);
extern void hostmux_poll(void);
extern void hostmux_deliver(__pdata uint16_t source_node, __xdata uint8_t * __pdata buf, __pdata uint8_t len);
extern uint8_t hostmux_get_next(__pdata uint8_t max_xmit, __xdata uint8_t * __pdata buf, __pdata uint16_t *destination);

#endif // _HOSTMUX_H_
