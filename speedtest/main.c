/**
 * Copyright (c) 2023 Pranit Arya
 * 
 * This demo does a simple speedtest using the nrf24l01 and the p78_nrf24 library. The MCU can either
 * be configured as a sender, which transmits packets and a reciever, which counts what was recieved
 * to calculate throughput in (k)B/s.
 * 
 * This demo doesn't do any checks on the packet sent or recieved (apart from hardware CRC on the nRF24)
 * other than using an 8-bit packet ID to keep track of packets.
 * 
 * This demo might be fun to play around with to see throughput depending on distance, LNA power, etc
 * 
 * This version for: Raspberry Pi Pico
 * 
 * TODO: Interrupts
 * 
 * MIT License
 */

// ---------------------------------
// User config
// ---------------------------------

// Uncomment based on device below:
#define RPI_PICO
// #define STM32F4

#define IS_SENDER 1 // Is this MCU sender or reciever

// ---------------------------------

#include "../p78_nrf24/nrf24l01.h"

#ifdef RPI_PICO

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#define RASPBERRYPI_PICO_W

#define LED_PORT NULL // Not needed for RPi Pico
#define LED_PIN 22 // PICO W LED
#define CE_PIN 21

#endif

#ifdef STM32F4

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#endif

// Data packet struct - 32 bit is max packet size for nRF24L01
typedef struct Packet {
    uint8_t id; // Keep track of packets
    uint8_t data[31]; // Data to send
    uint8_t len; // Not sent, just used in code
} packet_t;

// ---------------------------------
// nRF24 Configs
// ---------------------------------

const nrf24_pin_config_t nrf_pin_config = {
    .spi_periph = PICO_DEFAULT_SPI_INSTANCE,
    .csn_pin = PICO_DEFAULT_SPI_CSN_PIN, // GPIO pin for CSN
    .ce_pin = CE_PIN // GPIO pin for CE
};

const nrf24_config_t nrf_config = {
    .role = (IS_SENDER ? NRF24_ROLE_TX : NRF24_ROLE_RX), // NRF24_ROLE_TX or NRF24_ROLE_RX
    .rf_channel = 0x4C, // RF channel, 0x00 to 0x7F
    .power_level = NRF24_PWR_MIN, // Power level for LNA
    .enable_auto_ack = 0, // Enable auto ack TX/RX
    .rx_pipe = NRF24_PIPE_0,
    .data_rate = NRF24_DATA_RATE_2MBPS // Data rate
};

void err_handler() {
    while (true) {
        // uint8_t buff = 0xAA;
        // spi_write_blocking(PICO_DEFAULT_SPI_INSTANCE, &buff, 1);
        gpio_put(LED_PIN, 1);
        device_agnostic_sleep(250);
        gpio_put(LED_PIN, 0);
        device_agnostic_sleep(250);
    }
}

// Default packet to send
packet_t default_packet = {
    .id = 0, // Incremented each time
    .data = {0}, // Set to 0xAA in code
    .len = 32
};

uint32_t packets;
uint32_t last_packets_per_second;

// Device specific init - SPI, GPIO, etc

// Code for RPi Pico
#ifdef RPI_PICO
void mcu_init(void) {
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

    spi_init(PICO_DEFAULT_SPI_INSTANCE, 5 * 1000000); // 5 MHz
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    
    // CE pin setup
    gpio_init(CE_PIN);
    gpio_set_dir(CE_PIN, GPIO_OUT);
}
#endif

// Code for STM32F4, based on LibOpenCM3
#ifdef STM32F4
void mcu_init(void) {
    /* add your own code */

	// Clock setup
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_180MHZ]);

	// Enable SPI and GPIO clock
	rcc_periph_clock_enable(RCC_GPIOG);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_SPI5);

	// LED
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
	
	// SPI
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO7 | GPIO8 | GPIO9);
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);
	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO7 | GPIO9);
	spi_reset(SPI5);
	spi_init_master(SPI5,
		SPI_CR1_BAUDRATE_FPCLK_DIV_32,
		SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_1,
		SPI_CR1_DFF_8BIT,
		SPI_CR1_MSBFIRST);
	// SPI_CR2(SPI5) |= SPI_CR2_SSOE;
	// SPI_CR1(SPI5) = cr_tmp;
	// spi_set_nss_high(SPI5);
	spi_set_full_duplex_mode(SPI5);
	// spi_enable_ss_output(SPI5);
	// spi_enable_software_slave_management(SPI5);

	spi_enable(SPI5);
}
#endif

// --------------------------------------------------------------------
// Device agnostic functions help with compatibility between devices
// --------------------------------------------------------------------

void device_agnostic_sleep(uint32_t ms) {
    #ifdef RPI_PICO
    sleep_ms(ms);
    #endif
    #ifdef STM32F4
    for(int i = 0; i < (ms * 100000); i++) { // Not accurate, just gives ballpark figure
        __asm__("nop");
    }
    #endif
}

void device_agnostic_gpio_put(uint8_t port, uint8_t pin, uint8_t val) {
    #ifdef RPI_PICO
    gpio_put(pin, val);
    #endif

    #ifdef STM32F4
    if(val) {
        gpio_set(port, pin);
    } else {
        gpio_clear(port, pin);
    }
    #endif
}

// Timer callback for measuring packets per second
// Only applies to reciever side
void measure_timer_callback(void) {
    last_packets_per_second = packets;
    packets = 0;
    printf("Packets per second: %d\n", last_packets_per_second);
    printf("bps: %d\n", (last_packets_per_second * 32 * 8));
}

int main() {

    mcu_init();
    
    if(init_nrf24(&nrf_pin_config, &nrf_config) != 0) {
        err_handler();
    }

    volatile uint8_t data = 1;
    
	device_agnostic_gpio_put(LED_PORT, LED_PIN, 1);
    device_agnostic_sleep(250);
	device_agnostic_gpio_put(LED_PORT, LED_PIN, 0);

    // Set packet data to 0xAA - b10101010
    for(int i = 1; i < default_packet.len; i++) {
        default_packet.data[i] = 0xAA;
    }

    if(IS_SENDER) {
        while(1) {
            // Check if send FIFO empty
            read_mem(NRF24_R_FIFO_STATUS, &data, 1);
            if(!(data & 0x20)) {
                // Space in TX FIFO
                gpio_put(LED_PIN, 1);

                default_packet.id++;
                write_payload(&default_packet, default_packet.len);
            }
        }
    }
    else { // If receiver
        // Set up repeating timer to measure time (packets/s)
        struct repeating_timer timer;
        add_repeating_timer_ms(1000, &measure_timer_callback, NULL, &timer);

        while(1) {
            // Poll RX FIFO
            read_mem(NRF24_R_FIFO_STATUS, &data, 1);
            if(!(data & 0x01)) {
                // Read from FIFO
                read_payload(&data, 1);
                packets++;
            }
        }
    }
}