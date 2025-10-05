/*
 * Copyright 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT brcm_bcm2711_display

#include <errno.h>
#include <string.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bcm2711, CONFIG_DISPLAY_LOG_LEVEL);

struct bcm2711_display_config {
	uint32_t videocore_address;
	uint16_t height;
	uint16_t width;
};

typedef enum {
	RGB,
	BGR,
} RpiPixelOrder;

struct bcm2711_display_data {
	RpiPixelOrder pixel_order;

	mm_reg_t virtual_framebuffer_address;
	size_t framebuffer_size;
	uint32_t physical_width;
	uint32_t physical_height;
	uint32_t pitch;
};

// see raspberry pi firmware wiki(https://github.com/raspberrypi/firmware/wiki)
#define BCM2711_VIDEOCORE_MAILBOX_SIZE 0x40

static void mailbox_write(volatile uint32_t *mbox_reg_ptr, uint32_t value);
static uint32_t mailbox_read(volatile uint32_t *mbox_reg_ptr);
static int send_mailbox_for_vc(volatile uint32_t *mbox_reg_ptr, volatile uint32_t *mbox);
static void set_tag_4byte(volatile uint32_t *mbox, uint32_t tag, uint32_t val);
static void set_tag_8byte(volatile uint32_t *mbox, uint32_t tag, uint32_t val1, uint32_t val2);

static int bcm2711_display_init(const struct device *port)
{
	struct bcm2711_display_config *config = (struct bcm2711_display_config *)port->config;

	volatile uint32_t __attribute__((aligned(16))) mbox[35];
	mbox[0] = 35 * 4;
	mbox[1] = 0;
	set_tag_8byte(&mbox[2], 0x48003, config->width, config->height); // set phy wh
	set_tag_8byte(&mbox[7], 0x48004, config->width, config->height); // set virt wh
	set_tag_8byte(&mbox[12], 0x48009, 0, 0);                         // set virt offsetx
	set_tag_4byte(&mbox[17], 0x48005, 32);                           // set depth
	set_tag_4byte(&mbox[21], 0x48006, 1);                            // set pixel order
	set_tag_8byte(&mbox[25], 0x40001, 16, 0);                        // get framebuffer
	set_tag_4byte(&mbox[30], 0x40008, 0);                            // get pitch
	mbox[34] = 0;                                                    // terminate

	uintptr_t mbox_reg_virt_ptr;
	device_map(&mbox_reg_virt_ptr, config->videocore_address, BCM2711_VIDEOCORE_MAILBOX_SIZE,
		   K_MEM_CACHE_NONE);

	if (!send_mailbox_for_vc((volatile uint32_t *)mbox_reg_virt_ptr, mbox)) {
		LOG_ERR("Failed initialize display driver\n");
		return -EIO;
	}

	uint32_t depth = mbox[20];
	uint32_t framebuffer_address = mbox[28];
	if (depth != 32) {
		LOG_ERR("Unable to set screen resolution\n");
		return -EIO;
	}
	if (framebuffer_address == 0) {
		LOG_ERR("Failed framebuffer allocation\n");
		return -EIO;
	}

	struct bcm2711_display_data *data = (struct bcm2711_display_data *)port->data;
	data->pixel_order = mbox[24] ? RGB : BGR;
	data->physical_width = mbox[5];
	data->physical_height = mbox[6];
	data->framebuffer_size = mbox[29];
	data->pitch = mbox[33];

	device_map(&data->virtual_framebuffer_address, framebuffer_address & 0x3FFFFFFF, 0x300000,
		   K_MEM_CACHE_NONE);

	return 0;
}

static void set_tag_4byte(volatile uint32_t *mbox, uint32_t tag, uint32_t val)
{
	mbox[0] = tag;
	mbox[1] = 4;
	mbox[3] = val;
}

static void set_tag_8byte(volatile uint32_t *mbox, uint32_t tag, uint32_t val1, uint32_t val2)
{
	mbox[0] = tag;
	mbox[1] = 8;
	mbox[3] = val1;
	mbox[4] = val2;
}

static int send_mailbox_for_vc(volatile uint32_t *mbox_reg_ptr, volatile uint32_t *mbox)
{
	static const uint32_t CH_PROP_FOR_VC = 0x00000008;
	uint32_t mbox_data = (uint32_t)((uintptr_t)mbox & ~0xF) | CH_PROP_FOR_VC;

	mailbox_write(mbox_reg_ptr, mbox_data);
	while (1) {
		if (mailbox_read(mbox_reg_ptr) == mbox_data) {
			break;
		}
	}

	return mbox[1] == 0x80000000;
}

static void mailbox_write(volatile uint32_t *mbox_reg_ptr, uint32_t value)
{
	static const uint32_t MBOX_FULL = 0x80000000;
	while (*mbox_reg_ptr & MBOX_FULL) {
		__asm__ volatile("nop");
	}

	static const uint32_t WRITE_REG_OFFSET = 0x20 / 4;
	*(mbox_reg_ptr + WRITE_REG_OFFSET) = value;
}

static uint32_t mailbox_read(volatile uint32_t *mbox_reg_ptr)
{
	static const uint32_t MBOX_EMPTY = 0x40000000;
	static const uint32_t STATUS_REG_OFFSET = 0x18 / 4;
	while (*(mbox_reg_ptr + STATUS_REG_OFFSET) & MBOX_EMPTY) {
		__asm__ volatile("nop");
	}

	return *mbox_reg_ptr;
}

static int bcm2711_display_write(const struct device *dev, const uint16_t x, const uint16_t y,
				 const struct display_buffer_descriptor *desc, const void *buf)
{
	const char *pixel_ptr = (const char *)buf;
	const struct bcm2711_display_data *data = (struct bcm2711_display_data *)dev->data;

	char *frame_buffer = (char *)data->virtual_framebuffer_address;
	for (int h = 0; h < desc->height; ++h) {
		char *ptr = frame_buffer + (y + h) * data->pitch + x * 4;
		for (int w = 0; w < desc->width; ++w) {
			size_t buf_index = ((h * desc->pitch) + w) * 3;

			uint32_t pixel = (uint32_t)pixel_ptr[buf_index];
			pixel |= (uint32_t)pixel_ptr[buf_index + 1] << 8;
			pixel |= (uint32_t)pixel_ptr[buf_index + 2] << 16;
			*((uint32_t *)ptr) = pixel;
			ptr += 4;
		}
	}
	return 0;
}

static void bcm2711_display_get_capabilities(const struct device *dev,
					     struct display_capabilities *capabilities)
{
	const struct bcm2711_display_config *config = dev->config;
	capabilities->x_resolution = config->width;
	capabilities->y_resolution = config->height;
	capabilities->screen_info = 0;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888;
}

static int bcm2711_display_set_pixel_format(const struct device *dev,
					    const enum display_pixel_format pixel_format)
{
	if (pixel_format != PIXEL_FORMAT_RGB_888) {
		return -ENOTSUP;
	}
	return 0;
}

static const struct display_driver_api bcm2711_display_api = {
	.write = bcm2711_display_write,
	.get_capabilities = bcm2711_display_get_capabilities,
	.set_pixel_format = bcm2711_display_set_pixel_format,
};

#define BCM2711_PANEL(id)                                                                          \
	static const struct bcm2711_display_config bcm2711_config_##id = {                         \
		.videocore_address = DT_INST_REG_ADDR(id),                                         \
		.height = DT_INST_PROP(id, height),                                                \
		.width = DT_INST_PROP(id, width),                                                  \
	};                                                                                         \
	static struct bcm2711_display_data bcm2711_data_##id = {0};                                \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &bcm2711_display_init, NULL, &bcm2711_data_##id,                 \
			      &bcm2711_config_##id, POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,     \
			      &bcm2711_display_api);

DT_INST_FOREACH_STATUS_OKAY(BCM2711_PANEL)
