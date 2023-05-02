/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)

#define MY_SPI_SLAVE  DT_NODELABEL(my_spi_slave)

// SPI master functionality
const struct device *spi_dev;
static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);

struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};

static void spi_init(void)
{
	spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if(!device_is_ready(spi_dev)) {
		printk("SPI master device not ready!\n");
	}
	if(!device_is_ready(spim_cs.gpio.port)){
		printk("SPI master chip select device not ready!\n");
	}
}

static const struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 4000000,
	.slave = 0,
	.cs = &spim_cs,
};

static int spi_write_test_msg(void)
{
	static uint8_t counter = 0;
	static uint8_t tx_buffer[2];
	static uint8_t rx_buffer[2];

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	// Update the TX buffer with a rolling counter
	tx_buffer[0] = counter++;
	printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[1]);

	// Reset signal
	k_poll_signal_reset(&spi_done_sig);
	
	// Start transaction
	int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
		return error;
	}

	// Wait for the done signal to be raised and log the rx buffer
	int spi_signaled, spi_result;
	do{
		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
	} while(spi_signaled == 0);
	printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[0], rx_buffer[1]);
	return 0;
}

// SPI slave functionality
const struct device *spi_slave_dev;
static struct k_poll_signal spi_slave_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_slave_done_sig);

static const struct spi_config spi_slave_cfg = {
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_OP_MODE_SLAVE,
	.frequency = 4000000,
	.slave = 0,
};

static void spi_slave_init(void)
{
	spi_slave_dev = DEVICE_DT_GET(MY_SPI_SLAVE);
	if(!device_is_ready(spi_dev)) {
		printk("SPI slave device not ready!\n");
	}
}

static uint8_t slave_tx_buffer[2];
static uint8_t slave_rx_buffer[2];
static int spi_slave_write_test_msg(void)
{
	static uint8_t counter = 0;


	const struct spi_buf s_tx_buf = {
		.buf = slave_tx_buffer,
		.len = sizeof(slave_tx_buffer)
	};
	const struct spi_buf_set s_tx = {
		.buffers = &s_tx_buf,
		.count = 1
	};

	struct spi_buf s_rx_buf = {
		.buf = slave_rx_buffer,
		.len = sizeof(slave_rx_buffer),
	};
	const struct spi_buf_set s_rx = {
		.buffers = &s_rx_buf,
		.count = 1
	};

	// Update the TX buffer with a rolling counter
	slave_tx_buffer[1] = counter++;
	printk("SPI SLAVE TX: 0x%.2x, 0x%.2x\n", slave_tx_buffer[0], slave_tx_buffer[1]);

	// Reset signal
	k_poll_signal_reset(&spi_slave_done_sig);
	
	// Start transaction
	int error = spi_transceive_async(spi_slave_dev, &spi_slave_cfg, &s_tx, &s_rx, &spi_slave_done_sig);
	if(error != 0){
		printk("SPI slave transceive error: %i\n", error);
		return error;
	}
	return 0;
}

static int spi_slave_check_for_message(void)
{
	int signaled, result;
	k_poll_signal_check(&spi_slave_done_sig, &signaled, &result);
	if(signaled != 0){
		return 0;
	}
	else return -1;
}

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	spi_init();

	spi_slave_init();

	printk("SPI master/slave example started\n");
	
	spi_slave_write_test_msg();

	while (1) {
		spi_write_test_msg();
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);

		if(spi_slave_check_for_message() == 0){
			// Print the last received data
			printk("SPI SLAVE RX: 0x%.2x, 0x%.2x\n", slave_rx_buffer[0], slave_rx_buffer[1]);
			
			// Prepare the next SPI slave transaction
			spi_slave_write_test_msg();
		}
	}
}
