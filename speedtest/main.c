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

#include "pico/stdlib.h"
#include "../p78_nrf24/nrf24l01.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#define IS_SENDER 1 // Is this MCU sender or reciever

#define RASPBERRYPI_PICO_W

#define LED_PIN 22 // PICO W LED
#define CE_PIN 21

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

// Default packet to send
packet_t default_packet = {
    .id = 0, // Incremented each time
    .data = {0}, // Set to 0xAA in code
    .len = 32
};

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

    if(init_nrf24(&nrf_pin_config, &nrf_config) != 0) {
        err_handler();
    }

    volatile uint8_t data = 1;
    
	gpio_put(LED_PIN, 1);
    sleep_ms(250);
	gpio_put(LED_PIN, 0);

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
}