/**
 * Copyright (c) 2023 Pranit Arya
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "p78_nrf24/nrf24l01.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#define RASPBERRYPI_PICO_W

#define LED_PIN 22 // PICO W LED
#define CE_PIN 21

const nrf24_pin_config_t nrf_pin_config = {
    .spi_periph = PICO_DEFAULT_SPI_INSTANCE,
    .csn_pin = PICO_DEFAULT_SPI_CSN_PIN, // GPIO pin for CSN
    .ce_pin = CE_PIN // GPIO pin for CE
};

const nrf24_config_t nrf_config = {
    .role = NRF24_ROLE_TX, // NRF24_ROLE_TX or NRF24_ROLE_RX
    .rf_channel = 0x4C, // RF channel, 0x00 to 0x7F
    .power_level = NRF24_PWR_MIN, // Power level for LNA
    .enable_auto_ack = 0, // Enable auto ack TX/RX
    .rx_pipe = NRF24_PIPE_0
};

void err_handler() {
    while (true) {
        // uint8_t buff = 0xAA;
        // spi_write_blocking(PICO_DEFAULT_SPI_INSTANCE, &buff, 1);
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}

int main() {
    stdio_init_all();

    printf("Starting");

    // const uint LED_PIN = 0;
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);
    // while (true) {
    //     gpio_put(LED_PIN, 1);
    //     sleep_ms(250);
    //     gpio_put(LED_PIN, 0);
    //     sleep_ms(250);
    // }

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // err_handler();

    spi_init(PICO_DEFAULT_SPI_INSTANCE, 5 * 1000000); // 1 MHz
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    // bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    // Make the CS pin available to picotool
    // bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    // CE pin setup
    gpio_init(CE_PIN);
    gpio_set_dir(CE_PIN, GPIO_OUT);

    if(init_nrf24(&nrf_pin_config, &nrf_config) != 0) {
        err_handler();
    }

    volatile uint8_t data = 1;

	// write_mem(NRF24_R_RX_PW_P0, &data, 1);
	// data = (0x4C & 0x7F);
	// write_mem(NRF24_R_RF_CH, &data, 1);

	gpio_put(LED_PIN, 1);
	// for(volatile int i = 0; i < 5000; i++);
    sleep_ms(250);
	gpio_put(LED_PIN, 0);

	while(1) {
		// spi_send(SPI5, 0xA5);
		// read_payload(&data, 1);
		// read_mem(NRF24_R_FIFO_STATUS, &data, 1);
		// if(!(data & 0x01)) {
		// // 	// gpio_set(GPIOG, GPIO13);
		// 	// Read from FIFO
		// 	read_payload(&data, 1);
		// 	if(data) {
		// 		gpio_set(GPIOG, GPIO13);
		// 	}
		// 	else {
		// 		gpio_clear(GPIOG, GPIO13);
		// 	}

		// }
        
        read_mem(NRF24_R_FIFO_STATUS, &data, 1);
        if((data & 0x20) != 0) {
            gpio_put(LED_PIN, 1);
        }
        
        data = 1;
        write_payload(&data, 1);
        sleep_ms(1000);
        data = 0;
        write_payload(&data, 1);
        sleep_ms(1000);

		// if(!(data & (1 << 5))) {
		// 	data = 1;
		// 	write_payload(&data, 1);
		// }

		// for(volatile int i = 0; i < 1000000; i++);
		// else {
		// 	gpio_clear(GPIOG, GPIO13);
		// }
    }
}