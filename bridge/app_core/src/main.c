#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(bridge_app, LOG_LEVEL_INF);

int main(void)
{
	const struct device *leds = DEVICE_DT_GET(DT_NODELABEL(leds));
	const struct device *supply = DEVICE_DT_GET(DT_NODELABEL(led_supply));
	int err = usb_enable(NULL);

	if (err) {
		LOG_ERR("USB initialization failed: %d", err);
	}

	if (!device_is_ready(leds) || !device_is_ready(supply)) {
		LOG_ERR("LEDs or 5 V supply not ready");
		return -ENODEV;
	}

	/* No DTR wait: blink even without an open USB terminal. */
	while (true) {
		for (uint32_t i = 0; i < DT_CHILD_NUM(DT_NODELABEL(leds)); i++) {
			err = led_on(leds, i);
			if (err) {
				LOG_ERR("LED %u on failed: %d", i, err);
				return err;
			}
			k_msleep(300);
			err = led_off(leds, i);
			if (err) {
				LOG_ERR("LED %u off failed: %d", i, err);
				return err;
			}
			k_msleep(100);
		}
	}
}
