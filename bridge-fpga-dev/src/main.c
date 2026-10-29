#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/timing/timing.h>
#include <zephyr/usb/usb_device.h>
#include "fpga_gpio.h"
#include "fpga_spi.h"
#include "jtag.h"
#include "vme_image.h"

static const struct shell *active_shell;
void fpga_message(const char *text)
{
	shell_fprintf(active_shell, SHELL_NORMAL, "%s", text);
}
static int identify(const struct shell *sh)
{
	active_shell = sh;
	int result = fpga_gpio_init();
	uint32_t id = 0;
	if (!result) result = jtag_identify(&fpga_io, &id);
	shell_print(sh, "IDCODE=%08x expected=012b8043 chain-test=%s result=%d DONE=%d",
		    id, result ? "FAIL" : "PASS", result, result ? -1 : fpga_done());
	return result;
}
static int cmd_id(const struct shell *sh, size_t argc, char **argv)
{
	return identify(sh);
}
static int cmd_whoami(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t rx[3] = {0};
	/* TIMER2 includes any interrupt preemption during the polling transfer. */
	timing_init();
	timing_start();
	timing_t start = timing_counter_get();
	int err = fpga_spi_whoami(rx);
	timing_t end = timing_counter_get();
	timing_stop();
	uint64_t ns = timing_cycles_to_ns(timing_cycles_get(&start, &end));
	shell_print(sh, "SPI transfer: %u.%03u us (polling, includes CS delays)",
		(unsigned)(ns / 1000), (unsigned)(ns % 1000));
	if (err) {
		shell_error(sh, "FPGA WHOAMI: SPI %s (%d); reboot MCU after a transfer failure",
			err == -ETIMEDOUT ? "timeout" : "transfer failed", err);
		return err;
	}
	uint16_t whoami = ((uint16_t)rx[1] << 8) | rx[2];
	if (whoami != 0x10ac) {
		shell_error(sh, "FPGA WHOAMI: 0x%04X — FAIL (expected 0x10AC; RX: %02X %02X %02X)",
			whoami, rx[0], rx[1], rx[2]);
		return -EIO;
	}
	shell_print(sh, "FPGA WHOAMI: 0x%04X — PASS", whoami);
	return 0;
}
static int cmd_info(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Full VME 12.1, %u bytes, SHA256=%s", (unsigned)sizeof(vme_image), VME_SHA256);
	shell_print(sh, "CRC check=%d; Flash-write support=%s", vme_check(vme_image, sizeof(vme_image)),
		    IS_ENABLED(CONFIG_FPGA_FLASH_WRITE) ? "enabled" : "disabled");
	shell_print(sh, "Sequence includes SRAM/CF/UFM/Feature erase, 575 page writes, USERCODE,");
	shell_print(sh, "Feature Row=0000000000000000, FEABITS=0x0620, verify, DONE and ISC_DISABLE.");
	shell_print(sh, "No OTP/security opcodes. Application behavior is not proven by verification.");
	return 0;
}
static int cmd_program(const struct shell *sh, size_t argc, char **argv)
{
	if (!IS_ENABLED(CONFIG_FPGA_FLASH_WRITE)) {
		shell_error(sh, "Flash writes disabled in this build; first-write approval is required.");
		return -EACCES;
	}
	int result = vme_check(vme_image, sizeof(vme_image));
	if (result) {
		shell_error(sh, "VME validation failed (%d)", result);
		return result;
	}
	result = identify(sh);
	if (result) return result;
	shell_print(sh, "Starting approved VME erase/program/verify; do not interrupt power.");
	result = vme_run(vme_image, sizeof(vme_image), &fpga_io);
	shell_print(sh, "Programming %s (%d), DONE=%d. Reboot MCU before another attempt.",
		    result ? "FAILED" : "VERIFIED", result, fpga_done());
	return result;
}
SHELL_STATIC_SUBCMD_SET_CREATE(fpga_commands,
	SHELL_CMD_ARG(info, NULL, "Image and programming scope", cmd_info, 1, 0),
	SHELL_CMD_ARG(id, NULL, "Read-only IDCODE and chain check", cmd_id, 1, 0),
	SHELL_CMD_ARG(whoami, NULL, "Read application WHOAMI over SPI3", cmd_whoami, 1, 0),
	SHELL_CMD_ARG(program, NULL, "Program reviewed image", cmd_program, 1, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(fpga, &fpga_commands, "Bridge FPGA programmer", NULL);

int main(void)
{
	int spi_err = fpga_spi_init();
	if (spi_err) {
		printk("FPGA SPI initialization failed: %d\n", spi_err);
		return spi_err;
	}
	static const struct gpio_dt_spec buttons[] = {
		GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
		GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
	};

	/* Apply the DT pull-ups without interrupts, callbacks, or button actions. */
	for (unsigned i = 0; i < ARRAY_SIZE(buttons); i++) {
		int err = gpio_is_ready_dt(&buttons[i]) ?
			gpio_pin_configure_dt(&buttons[i], GPIO_INPUT) : -ENODEV;
		if (err) {
			printk("Button %u input initialization failed: %d\n", i, err);
			return err;
		}
	}
	return usb_enable(NULL);
}
