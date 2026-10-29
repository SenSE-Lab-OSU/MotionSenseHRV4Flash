#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/timing/timing.h>
#include <hal/nrf_spim.h>
#include <string.h>
#include "fpga_spi.h"

/* CONFIG_SPI=n: no Zephyr SPI3 device exists; this module alone owns SPIM3. */
PINCTRL_DT_DEFINE(DT_NODELABEL(spi3));
static NRF_SPIM_Type *const spim = (NRF_SPIM_Type *)DT_REG_ADDR(DT_NODELABEL(spi3));
static const struct gpio_dt_spec cs = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi3), cs_gpios, 0);
/* Persistent DMA storage remains valid even if STOPPED never arrives. */
static uint8_t tx_dma[3] = {0x0f, 0, 0}, rx_dma[3];
static bool ready, faulted;
BUILD_ASSERT(DT_PROP(DT_ALIAS(fpga), spi_max_frequency) == 8000000);

int fpga_spi_init(void)
{
	if (!gpio_is_ready_dt(&cs)) return -ENODEV;
	int err = gpio_pin_configure_dt(&cs, GPIO_OUTPUT_INACTIVE);
	if (err) return err;
	err = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(spi3)), PINCTRL_STATE_DEFAULT);
	if (err) return err;
	nrf_spim_int_disable(spim, UINT32_MAX);
	spim->SHORTS = 0;
	nrf_spim_configure(spim, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
	nrf_spim_frequency_set(spim, NRF_SPIM_FREQ_8M);
	ready = true;
	return 0;
}

static bool wait_event(nrf_spim_event_t event)
{
	timing_t start = timing_counter_get();
	while (!nrf_spim_event_check(spim, event)) {
		timing_t now = timing_counter_get();
		if (timing_cycles_get(&start, &now) >= timing_freq_get() / 1000) return false;
	}
	return true;
}

/* Caller owns the shell thread and runs the timing counter during this call. */
int fpga_spi_whoami(uint8_t rx[3])
{
	if (!ready) return -ENODEV;
	if (faulted) return -EIO;
	nrf_spim_event_clear(spim, NRF_SPIM_EVENT_END);
	nrf_spim_event_clear(spim, NRF_SPIM_EVENT_STOPPED);
	nrf_spim_tx_buffer_set(spim, tx_dma, sizeof(tx_dma));
	nrf_spim_rx_buffer_set(spim, rx_dma, sizeof(rx_dma));
	nrf_spim_enable(spim);
	int err = gpio_pin_set_dt(&cs, 1);
	if (!err) {
		k_busy_wait(1);
		__DMB();
		nrf_spim_task_trigger(spim, NRF_SPIM_TASK_START);
		if (!wait_event(NRF_SPIM_EVENT_END)) {
			err = -ETIMEDOUT;
			nrf_spim_task_trigger(spim, NRF_SPIM_TASK_STOP);
			if (!wait_event(NRF_SPIM_EVENT_STOPPED)) {
				/* Do not disable before STOPPED or reuse outstanding DMA. */
				faulted = true;
				err = -EIO;
			}
		} else if (nrf_spim_tx_amount_get(spim) != 3 || nrf_spim_rx_amount_get(spim) != 3) {
			err = -EIO;
		}
		k_busy_wait(1);
	}
	int cs_err = gpio_pin_set_dt(&cs, 0);
	if (!faulted) nrf_spim_disable(spim);
	__DMB();
	if (!err && !cs_err) memcpy(rx, rx_dma, sizeof(rx_dma));
	return err ? err : cs_err;
}
