#include "fpga_gpio.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define FPGA DT_ALIAS(fpga)
static const struct gpio_dt_spec outputs[] = {
	GPIO_DT_SPEC_GET(FPGA, tdi_gpios), GPIO_DT_SPEC_GET(FPGA, tck_gpios),
	GPIO_DT_SPEC_GET(FPGA, tms_gpios),
};
static const struct gpio_dt_spec tdo = GPIO_DT_SPEC_GET(FPGA, tdo_gpios);
static const struct gpio_dt_spec done = GPIO_DT_SPEC_GET(FPGA, done_gpios);
static const struct gpio_dt_spec program = GPIO_DT_SPEC_GET(FPGA, program_gpios);
static const struct gpio_dt_spec inputs[] = {
	GPIO_DT_SPEC_GET(FPGA, irq_gpios),
	GPIO_DT_SPEC_GET_BY_IDX(FPGA, io_gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(FPGA, io_gpios, 1),
};

int fpga_gpio_init(void)
{
	for (unsigned i = 0; i < ARRAY_SIZE(outputs); i++) {
		if (!gpio_is_ready_dt(&outputs[i])) return -ENODEV;
		int err = gpio_pin_configure_dt(&outputs[i], i == 1 ? GPIO_OUTPUT_INACTIVE : GPIO_OUTPUT_ACTIVE);
		if (err) return err;
	}
	const struct gpio_dt_spec *pins[] = {&tdo, &done, &program, &inputs[0], &inputs[1], &inputs[2]};
	for (unsigned i = 0; i < ARRAY_SIZE(pins); i++) {
		if (!gpio_is_ready_dt(pins[i])) return -ENODEV;
		int err = gpio_pin_configure_dt(pins[i], pins[i] == &program ? GPIO_OUTPUT_INACTIVE : GPIO_INPUT);
		if (err) return err;
	}
	return 0;
}
static int write_pins(unsigned char mask, unsigned char value)
{
	if (mask & ~7) return -EINVAL;
	for (unsigned i = 0; i < ARRAY_SIZE(outputs); i++) if (mask & BIT(i)) {
		int err = gpio_pin_set_dt(&outputs[i], !!value);
		if (err) return err;
	}
	return 0;
}
static int read_tdo(void) { return gpio_pin_get_dt(&tdo); }
int fpga_done(void) { return gpio_pin_get_dt(&done); }
static void delay_us(uint32_t us)
{
	if (us >= 1000) k_usleep(us);
	else k_busy_wait(us);
}
static uint64_t now_ms(void) { return k_uptime_get(); }
const struct vme_io fpga_io = {write_pins, read_tdo, delay_us, now_ms, fpga_message, NULL};
