/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

static uart_inst_t *uart_instance;
static int uart_irq;

#if PICO_NO_BI_STDIO_UART
#define stdio_bi_decl_if_func_used(x)
#else
#define stdio_bi_decl_if_func_used bi_decl_if_func_used
#endif

void uart_interrupt_handler(void);
void uart_prepare_to_await_input(void);

void stdio_uart_init() {
#ifdef uart_default
    int tx_pin = -1;
    int rx_pin = -1;
#ifdef PICO_DEFAULT_UART_TX_PIN
    tx_pin = PICO_DEFAULT_UART_TX_PIN;
#ifdef PICO_DEFAULT_UART_RX_PIN
    rx_pin = PICO_DEFAULT_UART_RX_PIN;
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdin / stdout"));
    bi_decl_if_func_used(bi_2pins_with_func(PICO_DEFAULT_UART_RX_PIN, PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART));
#else
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdout"));
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART));
#endif
#elif defined(PICO_DEFAULT_UART_RX_PIN)
    rx_pin = PICO_DEFAULT_UART_RX_PIN;
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdin"));
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART));
#endif
#if !defined(PICO_DEFAULT_UART_BAUD_RATE)
    panic("UART baud rate undefined");
#else
    stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, tx_pin, rx_pin);
#endif
#endif
}

void stdout_uart_init() {
#if defined(uart_default) && defined(PICO_DEFAULT_UART_TX_PIN)
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART));
#if !defined(PICO_DEFAULT_UART_BAUD_RATE)
    panic("UART baud rate undefined");
#else
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdout"));
    stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, PICO_DEFAULT_UART_TX_PIN, -1);
#endif
#endif
}

void stdin_uart_init() {
#if defined(uart_default) && defined(PICO_DEFAULT_UART_RX_PIN)
    bi_decl_if_func_used(bi_1pin_with_func(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART));
#if !defined(PICO_DEFAULT_UART_BAUD_RATE)
    panic("UART baud rate undefined");
#else
    stdio_bi_decl_if_func_used(bi_program_feature("UART stdin"));
    stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, -1, PICO_DEFAULT_UART_RX_PIN);
#endif
#endif
}

void stdio_uart_init_full(struct uart_inst *uart, uint baud_rate, int tx_pin, int rx_pin) {
    uart_instance = uart;
    uart_init(uart_instance, baud_rate);
    if (tx_pin >= 0) gpio_set_function((uint)tx_pin, GPIO_FUNC_UART);
    if (rx_pin >= 0) gpio_set_function((uint)rx_pin, GPIO_FUNC_UART);
    stdio_set_driver_enabled(&stdio_uart, true);
}

void stdio_uart_init_blocking(struct uart_inst *uart, uint8_t irq_priority) {
    assert(uart == uart_instance);
    uart_irq = uart == uart0 ? UART0_IRQ : UART1_IRQ;

    irq_add_shared_handler(uart_irq, uart_interrupt_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_priority(uart_irq, irq_priority);
    irq_set_enabled(uart_irq, true);
    
    // Was null before to indicate to stdio_get_until() that blocking wasn't supported.
    stdio_uart.prepare_to_await_input = uart_prepare_to_await_input;
}

static void stdio_uart_out_chars(const char *buf, int length) {
    uart_hw_t* hw = (uart_hw_t*) uart_instance;

    for (int i = 0; i <length; i++) {
        if (stdio_uart.prepare_to_await_input) {
            while (!uart_is_writable(uart_instance)) {
                hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
                stdio_wait();
            }
        }
        uart_putc(uart_instance, buf[i]);
    }
}

int stdio_uart_in_chars(char *buf, int length) {
    int i=0;
    while (i<length && uart_is_readable(uart_instance)) {
        buf[i++] = uart_getc(uart_instance);
    }
    return i ? i : PICO_ERROR_NO_DATA;
}

void uart_interrupt_handler(void) {
    uart_hw_t* hw = (uart_hw_t*) uart_instance;
    if (uart_is_readable(uart_instance)) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
    }
    if (uart_is_writable(uart_instance)) {
        hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }

    stdio_signal_from_isr();
}

void uart_prepare_to_await_input(void) {
    uart_hw_t* hw = (uart_hw_t*) uart_instance;
    hw_set_bits(&hw->imsc, UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
}

stdio_driver_t stdio_uart = {
    .out_chars = stdio_uart_out_chars,
    .in_chars = stdio_uart_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_UART_DEFAULT_CRLF
#endif
};
