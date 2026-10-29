#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/shell/shell.h>
#include <zephyr/timing/timing.h>
#include <zephyr/usb/usb_device.h>
#include "fpga_gpio.h"
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
	/* SPI3, 8 MHz from DT, mode 0, MSB first, GPIO CS with 1 us setup. */
	static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(DT_ALIAS(fpga),
		SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 1);
	uint8_t tx[] = {0x0f, 0x00, 0x00};
	uint8_t rx[3] = {0};
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	if (!spi_is_ready_dt(&spi)) {
		shell_error(sh, "FPGA WHOAMI: SPI3 or CS GPIO not ready");
		return -ENODEV;
	}
	int err = gpio_pin_set_dt(&spi.config.cs.gpio, 0);
	if (err) {
		shell_error(sh, "FPGA WHOAMI: cannot set CS high (%d)", err);
		return err;
	}
	/* One full-duplex transfer: CS stays low for all 24 clocks. The synchronous
	 * nrfx driver finishes or aborts the transfer before returning. */
	/* Nordic timing uses TIMER2 at 16 MHz, including time blocked on SPI. */
	timing_init();
	timing_start();
	timing_t start = timing_counter_get();
	err = spi_transceive_dt(&spi, &tx_set, &rx_set);
	timing_t end = timing_counter_get();
	timing_stop();
	int cs_err = gpio_pin_set_dt(&spi.config.cs.gpio, 0);
	uint64_t ns = timing_cycles_to_ns(timing_cycles_get(&start, &end));
	shell_print(sh, "SPI transfer: %u.%03u us (includes driver and CS delays)",
		(unsigned)(ns / 1000), (unsigned)(ns % 1000));
	if (cs_err) shell_error(sh, "FPGA WHOAMI: cannot restore CS high (%d)", cs_err);
	if (err) {
		shell_error(sh, "FPGA WHOAMI: SPI %s (%d)",
			err == -ETIMEDOUT ? "timeout" : "transfer failed", err);
		return err;
	}
	if (cs_err) return cs_err;
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
