/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2018, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <assert.h>

#include "applet.h"
#include "gpio/pio.h"
#include "trace.h"

#if 0
int gpio_configure(uint32_t index, uint8_t type, uint32_t attribute, struct _pin *pin_out);
{
	pin_out->group = index / 32;
	pin_out->mask = 1u << (index % 32);
	pin_out->type = type;
	pin_out->attribute = attribute;
}
#endif

void gpio_mailbox_to_pin(union gpio_mailbox *mailbox, struct _pin *pin);

void gpio_mailbox_to_pin(union gpio_mailbox *mailbox, struct _pin *pin)
{
	pin->group	 	= mailbox->in.group;
	pin->mask	  	= mailbox->in.mask;
	pin->type	  	= mailbox->in.type;
	pin->attribute 	= mailbox->in.attribute;
}

/*----------------------------------------------------------------------------
 *         Private functions
 *----------------------------------------------------------------------------*/

static uint32_t handle_cmd_initialize(uint32_t cmd, uint32_t *mailbox)
{
	union initialize_mailbox *mbx = (union initialize_mailbox*)mailbox;

	assert(cmd == APPLET_CMD_INITIALIZE);

	if (!applet_set_init_params(mbx))
		return APPLET_FAIL;

	trace_warning_wp("\r\nApplet 'GPIO' from "
			"softpack " SOFTPACK_VERSION ".\r\n");

	mbx->out.buf_addr = 0;
	mbx->out.buf_size = 0;
	mbx->out.page_size = 0;
	mbx->out.mem_size = 0;
	mbx->out.erase_support = 0;
	mbx->out.nand_header = 0;

	return APPLET_SUCCESS;
}

static uint32_t handle_cmd_gpio_configure(uint32_t cmd, uint32_t *mailbox)
{
	union gpio_mailbox *mbx = (union gpio_mailbox*)mailbox;
	struct _pin pin;

	assert(cmd == APPLET_CMD_GPIO_CFG);

	gpio_mailbox_to_pin(mbx, &pin);
	pio_configure(&pin, 1);

	trace_info("GPIO configure. group: %c, mask: 0x%lx"
			", type: %d, attribute: %ld\r\n",
			pin.group + 'A', pin.mask, pin.type, pin.attribute);

	return APPLET_SUCCESS;
}

static uint32_t handle_cmd_gpio_set(uint32_t cmd, uint32_t *mailbox)
{
	union gpio_mailbox *mbx = (union gpio_mailbox*)mailbox;
	struct _pin pin;

	assert(cmd == APPLET_CMD_GPIO_SET);

	gpio_mailbox_to_pin(mbx, &pin);
	pio_set(&pin);

	trace_info("GPIO set. group: %c, mask: 0x%lx\r\n",
			pin.group, pin.mask);

	return APPLET_SUCCESS;
}

static uint32_t handle_cmd_gpio_clear(uint32_t cmd, uint32_t *mailbox)
{
	union gpio_mailbox *mbx = (union gpio_mailbox*)mailbox;
	struct _pin pin;

	assert(cmd == APPLET_CMD_GPIO_CLEAR);

	gpio_mailbox_to_pin(mbx, &pin);
	pio_clear(&pin);

	trace_info("GPIO clear. group: %c, mask: 0x%lx\r\n",
			pin.group, pin.mask);

	return APPLET_SUCCESS;
}

static uint32_t handle_cmd_gpio_get(uint32_t cmd, uint32_t *mailbox)
{
	union gpio_mailbox *mbx = (union gpio_mailbox*)mailbox;
	struct _pin pin;

	assert(cmd == APPLET_CMD_GPIO_GET);

	gpio_mailbox_to_pin(mbx, &pin);
	mbx->out.value = pio_get(&pin);

	trace_info("GPIO get. group: %c, mask: 0x%lx => value: 0x%lx\r\n",
			pin.group + 'A', pin.mask, mbx->out.value);

	return APPLET_SUCCESS;
}

/*----------------------------------------------------------------------------
 *         Commands list
 *----------------------------------------------------------------------------*/

const struct applet_command applet_commands[] = {
	{ APPLET_CMD_INITIALIZE, handle_cmd_initialize },
	{ APPLET_CMD_GPIO_CFG, handle_cmd_gpio_configure },
	{ APPLET_CMD_GPIO_SET, handle_cmd_gpio_set },
	{ APPLET_CMD_GPIO_CLEAR, handle_cmd_gpio_clear },
	{ APPLET_CMD_GPIO_GET, handle_cmd_gpio_get },
	{ 0, NULL }
};
