#include "tty_acm.h"

#include "py/runtime.h"
#include "py/ringbuf.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/mutex.h"
#include "tusb.h"


#define _UART1_TX_PIN (4)
#define _UART1_RX_PIN (5)

#define _UART1_DTR_PIN (2)
#define _UART1_RTS_PIN (3)


#define _CDC_UART       (uart1)
#define _CDC_TUD_ITF    (1)
#define _CDC_UART_IRQ   (UART1_IRQ)

static alarm_id_t break_alarm = 0;

ringbuf_t uart2usb_buf;
auto_init_mutex(uart2usb_mutex);

static int64_t _break_timer_cb(alarm_id_t id, void *user_data) {
    break_alarm = 0;
    uart_set_break(_CDC_UART, false);
    (void) id;
    (void) user_data;
    return 0;
}

static inline void _break_timer_reset(uint16_t ms) {
    if (break_alarm > 0) {
        cancel_alarm(break_alarm);
        break_alarm = 0;
    }
    if (ms > 0) {
        break_alarm = add_alarm_in_ms(ms, _break_timer_cb, NULL, true);
    }
}

static inline unsigned _drain_uart() {
    unsigned transferred = 0;
    while (uart_is_readable(_CDC_UART) && ringbuf_free(&uart2usb_buf) > 0) {
        char c = uart_getc(_CDC_UART);
        ringbuf_put(&uart2usb_buf, c);
        transferred++;
    }
    return transferred;
}

STATIC mp_sched_node_t tty_acm_uart2usb_sched_node;

STATIC void tty_acm_flush_uart2usb(struct _mp_sched_node_t *node) {
    uint8_t buf[CFG_TUD_CDC_EP_BUFSIZE];
    unsigned i = 0;

    mutex_enter_blocking(&uart2usb_mutex);
    _drain_uart();
    while (ringbuf_avail(&uart2usb_buf) > 0 && i < (CFG_TUD_CDC_EP_BUFSIZE-1)) {
        buf[i++] = ringbuf_get(&uart2usb_buf);
    }
    mutex_exit(&uart2usb_mutex);

    if (i == 0) return;

    while (tud_cdc_n_write_available(_CDC_TUD_ITF) < i) {
        tud_task();
        tud_cdc_n_write_flush(_CDC_TUD_ITF);
    }
    tud_cdc_n_write(_CDC_TUD_ITF, buf, i);
    tud_cdc_n_write_flush(_CDC_TUD_ITF);

    mp_sched_schedule_node(&tty_acm_uart2usb_sched_node, tty_acm_flush_uart2usb);

    (void) node;
}

void tty_acm_uart_irq_handler(void) {
    if (mutex_enter_timeout_ms(&uart2usb_mutex, 0)) {
        _drain_uart();
        mp_sched_schedule_node(&tty_acm_uart2usb_sched_node, tty_acm_flush_uart2usb);
        mutex_exit(&uart2usb_mutex);
    }
}

void init_cdc_uart(void) {
    // Initialise UART 0
    uart_init(_CDC_UART, 115200);
    uart_set_format(_CDC_UART, 8 /* bits */, 0 /* stop */, 0 /* parity */);
    uart_set_fifo_enabled(_CDC_UART, true);

    gpio_set_function(_UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(_UART1_RX_PIN, GPIO_FUNC_UART);

    gpio_set_function(_UART1_DTR_PIN, GPIO_FUNC_SIO);
    gpio_put(_UART1_DTR_PIN, true);
    gpio_set_dir(_UART1_DTR_PIN, GPIO_OUT);

    gpio_set_function(_UART1_RTS_PIN, GPIO_FUNC_SIO);
    gpio_put(_UART1_RTS_PIN, true);
    gpio_set_dir(_UART1_RTS_PIN, GPIO_OUT);

    uart_set_hw_flow(_CDC_UART, false, false);

    ringbuf_alloc(&uart2usb_buf, 1024);

    irq_set_exclusive_handler(_CDC_UART_IRQ, tty_acm_uart_irq_handler);
    irq_set_enabled(_CDC_UART_IRQ, true);

    // Enable the uart irq; this macro sets the rx irq level to 4.
    uart_set_irq_enables(_CDC_UART, true, false);

}

void tty_acm_rx_cb(uint8_t itf) {
    if (itf != 1) return;
    while (tud_cdc_n_available(itf)) {
        int32_t data_char = tud_cdc_n_read_char(itf);
        if (data_char < 0) break;
        uart_putc_raw(_CDC_UART, data_char);
    }
}

// Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    // mp_printf(MP_PYTHON_PRINTER, "tud_cdc_line_state_cb(itf=%u, dtr=%d, rts=%d)\n", itf, dtr, rts);
    if (itf != 1) return;

    gpio_put(_UART1_DTR_PIN, !dtr);
    gpio_put(_UART1_RTS_PIN, !rts);

}

static inline uint databits_usb2uart(uint8_t data_bits)
{
	switch (data_bits) {
		case 5: return 5;
		case 6:	return 6;
		case 7:	return 7;
		default: return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1: return UART_PARITY_ODD;
		case 2: return UART_PARITY_EVEN;
		default: return UART_PARITY_NONE;
	}
}

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2: return 2;
		default: return 1;
	}
}



// Invoked when line coding is change via SET_LINE_CODING
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* lc) {
    // mp_printf(MP_PYTHON_PRINTER,"tud_cdc_line_coding_cb(itf=%u, \n", itf);
    // mp_printf(MP_PYTHON_PRINTER,"  bit_rate=%u\n", lc->bit_rate);
    // mp_printf(MP_PYTHON_PRINTER,"  stop_bits=%u\n", lc->stop_bits);
    // mp_printf(MP_PYTHON_PRINTER,"  parity=%u\n", lc->parity);
    // mp_printf(MP_PYTHON_PRINTER,"  data_bits=%u\n", lc->data_bits);

    if (itf != 1) return;

    uart_set_format(_CDC_UART, databits_usb2uart(lc->data_bits),
        stopbits_usb2uart(lc->stop_bits), parity_usb2uart(lc->parity));
    unsigned real_baud = uart_set_baudrate(_CDC_UART, lc->bit_rate);

    // if (b != lc->bit_rate) {
    //     mp_printf(MP_PYTHON_PRINTER,"baud=%u != %u\n", real_baud, lc->bit_rate);
    // }
    (void) real_baud;
}

// Invoked when received send break
void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms) {
    // mp_printf(MP_PYTHON_PRINTER,"tud_cdc_send_break_cb(itf=%u, t=%u ms)\n", itf, duration_ms);
    if (itf != 1) return;
    uart_set_break(_CDC_UART, true);
    _break_timer_reset(duration_ms);
}