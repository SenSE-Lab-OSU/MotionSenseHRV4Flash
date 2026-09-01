#include "ecgRecorder.h"

#include "BLEService.h"
#include "drivers/ecg/max30001.h"
#include "ecgRecordFormat.h"
#include "zephyrfilesystem.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ecg_recorder, CONFIG_LOG_LEVEL_MAX30001);

#define ECG_RECORD_MAX30001_NODE DT_ALIAS(ecg)

#define ECG_RECORD_THREAD_STACK_SIZE 4096
#define ECG_RECORD_THREAD_PRIORITY 5
#define ECG_RECORD_ANCHOR_TIMEOUT_MS 100

static const struct gpio_dt_spec ecg_intb =
	GPIO_DT_SPEC_GET(ECG_RECORD_MAX30001_NODE, intb_gpios);
static const struct gpio_dt_spec ecg_intb2 =
	GPIO_DT_SPEC_GET(ECG_RECORD_MAX30001_NODE, intb2_gpios);

static K_SEM_DEFINE(ecg_record_start_sem, 0, 1);
static K_SEM_DEFINE(ecg_fifo_sem, 0, 1);
static K_SEM_DEFINE(ecg_record_stopped_sem, 0, 1);
static K_SEM_DEFINE(ecg_anchor_sem, 0, 1);

static struct gpio_callback ecg_intb_callback;
static struct gpio_callback ecg_intb2_callback;
static atomic_t ecg_record_requested;
static atomic_t ecg_record_active;
static bool ecg_intb_callback_added;
static bool ecg_intb2_callback_added;
static atomic_t ecg_anchor_state;
static atomic_t ecg_anchor_error;
static uint32_t ecg_anchor_rtc_tick;
static uint32_t ecg_next_rtc_tick;

enum ecg_record_anchor_state {
	ECG_RECORD_ANCHOR_IDLE = 0,
	ECG_RECORD_ANCHOR_WAITING,
	ECG_RECORD_ANCHOR_CAPTURING,
	ECG_RECORD_ANCHOR_CAPTURED,
	ECG_RECORD_ANCHOR_ERROR,
};


static void ecg_record_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_DEFINE(ecg_record_thread_id,
		ECG_RECORD_THREAD_STACK_SIZE,
		ecg_record_thread,
		NULL,
		NULL,
		NULL,
		ECG_RECORD_THREAD_PRIORITY,
		0,
		0);

/**
 * @brief GPIO interrupt handler for the MAX30001 INTB line.
 *
 * Runs in interrupt context when the MAX30001 asserts INTB (the ECG FIFO has
 * reached its configured threshold). It does the minimum possible work:
 * giving ecg_fifo_sem to wake the recorder thread, which performs the actual
 * SPI FIFO drain in thread context where blocking is allowed.
 *
 * @param port GPIO port device (unused).
 * @param cb   Callback structure (unused).
 * @param pins Bitmask of triggering pins (unused).
 */
static void ecg_record_intb_handler(const struct device *port,
				    struct gpio_callback *cb,
				    uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_sem_give(&ecg_fifo_sem);
}

/**
 * @brief Latch the RTC tick for the first post-SYNCH ECG SAMP pulse.
 *
 * INT2B is configured as a self-clearing SAMP output. The MAX30001 emits the
 * pulse when the corresponding filtered ECG sample is placed in the FIFO,
 * making this ISR the only point where the FIFO sample timeline is tied to
 * the collection RTC. The atomic state transition rejects any extra pulses
 * that arrive before the recorder thread has masked SAMP at the sensor.
 */
static void ecg_record_intb2_handler(const struct device *port,
				     struct gpio_callback *cb,
				     uint32_t pins)
{
	uint32_t rtc_tick;
	int ret;

	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (!atomic_cas(&ecg_anchor_state, ECG_RECORD_ANCHOR_WAITING,
			ECG_RECORD_ANCHOR_CAPTURING)) {
		return;
	}

	ret = rtc0_collection_counter_get(&rtc_tick);
	if (ret != 0) {
		atomic_set(&ecg_anchor_error, ret);
		atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_ERROR);
	} else {
		ecg_anchor_rtc_tick = rtc_tick;
		atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_CAPTURED);
	}

	k_sem_give(&ecg_anchor_sem);
}

/**
 * @brief Prepare the MAX30001 INTB GPIO for use as a data-ready interrupt.
 *
 * Configures the INTB pin (taken from the max30001 devicetree node) as an
 * input and registers ecg_record_intb_handler() as its callback. The
 * callback is only added once per boot (tracked by ecg_intb_callback_added)
 * so repeated start/stop cycles do not accumulate duplicate callbacks. The
 * pin interrupt itself is left DISABLED; ecg_record_run() enables edge
 * triggering only after the sensor is fully configured, avoiding spurious
 * wakeups during bring-up.
 *
 * @retval 0 on success.
 * @retval -ENODEV if the GPIO controller is not ready.
 * @retval Other negative errno from GPIO configuration calls.
 */
static int ecg_record_configure_intb(void)
{
	int ret;

	if (!gpio_is_ready_dt(&ecg_intb)) {
		LOG_ERR("MAX30001 INTB GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&ecg_intb, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	if (!ecg_intb_callback_added) {
		gpio_init_callback(&ecg_intb_callback,
				   ecg_record_intb_handler,
				   BIT(ecg_intb.pin));

		ret = gpio_add_callback(ecg_intb.port, &ecg_intb_callback);
		if (ret != 0) {
			return ret;
		}
		ecg_intb_callback_added = true;
	}

	return gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
}

/**
 * @brief Prepare the MAX30001 INT2B GPIO for the one-shot SAMP anchor.
 */
static int ecg_record_configure_intb2(void)
{
	int ret;

	if (!gpio_is_ready_dt(&ecg_intb2)) {
		LOG_ERR("MAX30001 INT2B GPIO is not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&ecg_intb2, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	if (!ecg_intb2_callback_added) {
		gpio_init_callback(&ecg_intb2_callback,
				   ecg_record_intb2_handler,
				   BIT(ecg_intb2.pin));

		ret = gpio_add_callback(ecg_intb2.port, &ecg_intb2_callback);
		if (ret != 0) {
			return ret;
		}
		ecg_intb2_callback_added = true;
	}

	return gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_DISABLE);
}

static void ecg_record_disable_gpio_interrupts(void)
{
	(void)gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_DISABLE);
	(void)gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_DISABLE);
}

static int ecg_record_wait_for_anchor(uint32_t *rtc_tick)
{
	int ret;

	ret = k_sem_take(&ecg_anchor_sem, K_MSEC(ECG_RECORD_ANCHOR_TIMEOUT_MS));
	if (atomic_get(&ecg_record_requested) == 0) {
		return -ECANCELED;
	}
	if (ret != 0) {
		return -ETIMEDOUT;
	}

	if (atomic_get(&ecg_anchor_state) == ECG_RECORD_ANCHOR_CAPTURED) {
		*rtc_tick = ecg_anchor_rtc_tick;
		return 0;
	}
	if (atomic_get(&ecg_anchor_state) == ECG_RECORD_ANCHOR_ERROR) {
		return (int)atomic_get(&ecg_anchor_error);
	}

	return -EIO;
}

/**
 * @brief Serialize one ECG sample into a framed record and queue it for
 *        storage.
 *
 * Packs the sample into a fixed 12-byte frame designed to be robust when
 * read back from raw storage:
 *
 *   [0]  sync byte 0xA5        [1]  sync byte 0xEC
 *   [2]  record type (0x01)    [3]  ETAG (bits 0-2) | PTAG (bits 3-5)
 *   [4-7]  32-bit 512 Hz collection RTC tick, little-endian
 *   [8-10] 24-bit raw ECG sample, big-endian
 *   [11] CRC-8 over bytes 2-10
 *
 * The two sync bytes let a parser resynchronize mid-stream and the CRC
 * catches corruption. The timestamp is assigned when this time-valid FIFO
 * sample is persisted, not when it was read from the batched FIFO.
 *
 * @param sample Decoded ECG sample to store.
 * @param rtc_tick Collection RTC tick associated with sample.
 */
static void ecg_record_store_sample(const struct max30001_ecg_sample *sample,
				    uint32_t rtc_tick)
{
	uint8_t frame[ECG_RECORD_FORMAT_FRAME_BYTES];

	ecg_record_format_build_sample_frame(frame, sample->etag, sample->ptag,
					 sample->raw, rtc_tick);

	store_data(frame, sizeof(frame), ecg);
}

/**
 * @brief Store a batch of ECG samples read from the FIFO.
 *
 * Iterates over a batch produced by max30001_ecg_read_fifo(), skipping any
 * sample whose time_valid flag is false (words that do not represent a real
 * sample slot), and writes each remaining sample to storage via
 * ecg_record_store_sample(). The RTC tick advances once per stored time-valid
 * sample, independent of how many samples were batched before this drain.
 *
 * @param samples Array of decoded samples.
 * @param count   Number of valid entries in samples.
 */
static void ecg_record_process_samples(const struct max30001_ecg_sample *samples,
				       size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (!samples[i].time_valid) {
			continue;
		}

		ecg_record_store_sample(&samples[i], ecg_next_rtc_tick);
		ecg_next_rtc_tick++;
	}
}



/**
 * @brief Empty the MAX30001 FIFO and dispatch the samples.
 *
 * Called from the recorder thread each time the INTB interrupt fires (and
 * once more at shutdown). Reads the FIFO in bursts of up to
 * MAX30001_ECG_FIFO_MAX_SAMPLES, for at most 4 passes, stopping early when a
 * read returns fewer than a full burst or ends on an EOF-tagged sample —
 * both signs the FIFO is empty. The pass limit bounds time spent here if
 * samples arrive as fast as they are drained.
 *
 * Each burst is persisted via ecg_record_process_samples(). No IMU or
 * collection-progress data is transmitted over BLE from this path.
 *
 * FIFO read errors are returned to the recording-session supervisor. A FIFO
 * overflow means the sample timeline has a gap, so the supervisor restarts
 * the MAX30001 and captures a new SAMP/RTC anchor instead of fabricating
 * consecutive timestamps across the loss.
 */
static int ecg_record_drain_fifo(void)
{
	int ret;

	for (int pass = 0; pass < 4; pass++) {
		struct max30001_ecg_sample samples[MAX30001_ECG_FIFO_MAX_SAMPLES];
		size_t count = 0;

		ret = max30001_ecg_read_fifo(samples, ARRAY_SIZE(samples), &count);
		if (ret != 0) {
			LOG_ERR("MAX30001 ECG FIFO read failed: %d", ret);
			return ret;
		}

		if (count == 0) {
			return 0;
		}

		ecg_record_process_samples(samples, count);

		if (count < ARRAY_SIZE(samples) || samples[count - 1].eof) {
			return 0;
		}
	}

	return 0;
}

/**
 * @brief Execute one complete ECG recording session.
 *
 * The main body of a recording configures both MAX30001 interrupt GPIOs and
 * captures the first post-SYNCH SAMP pulse on INT2B. That pulse provides the
 * RTC tick for the first FIFO sample; every later time-valid FIFO sample is
 * assigned the next tick. Once anchored, SAMP is masked and only the 16-word
 * FIFO watermark interrupt remains active during steady-state recording.
 *
 * On a normal stop, the interrupts are disabled, one final FIFO drain captures
 * remaining samples, and the sensor is powered down. Setup and FIFO failures
 * skip that drain so no sample is written without a valid, continuous timing
 * base. Requires the NAND filesystem to be ready before starting.
 *
 * @retval 0 on a clean stop.
 * @retval -ENODEV if the filesystem is not ready.
 * @retval Other negative errno if sensor or GPIO setup fails.
 */
static int ecg_record_run(void)
{
	int ret;
	int stop_ret;
	uint32_t rtc_tick;
	bool sensor_configured = false;
	bool anchor_ready = false;

	if (!file_system_ready) {
		LOG_ERR("Filesystem is not ready for ECG recording");
		return -ENODEV;
	}

	ret = ecg_record_configure_intb();
	if (ret != 0) {
		LOG_ERR("Failed to configure MAX30001 INTB: %d", ret);
		return ret;
	}

	ret = ecg_record_configure_intb2();
	if (ret != 0) {
		LOG_ERR("Failed to configure MAX30001 INT2B: %d", ret);
		return ret;
	}

	k_sem_reset(&ecg_fifo_sem);
	k_sem_reset(&ecg_anchor_sem);
	atomic_set(&ecg_anchor_error, 0);
	atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_WAITING);
	ecg_anchor_rtc_tick = 0U;
	ecg_next_rtc_tick = 0U;

	/* Fail before touching the sensor if collection timing is not active. */
	ret = rtc0_collection_counter_get(&rtc_tick);
	if (ret != 0) {
		LOG_ERR("Collection RTC is unavailable for ECG timing: %d", ret);
		goto out;
	}

	sensor_configured = true;
	ret = max30001_ecg_init_512();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG init failed: %d", ret);
		goto out;
	}

	/* Both GPIOs are armed while their MAX30001 sources remain masked. */
	ret = gpio_pin_interrupt_configure_dt(&ecg_intb, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to enable MAX30001 INTB interrupt: %d", ret);
		goto out;
	}

	ret = gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to enable MAX30001 INT2B interrupt: %d", ret);
		goto out;
	}

	ret = max30001_ecg_start();
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG start failed: %d", ret);
		goto out;
	}

	ret = ecg_record_wait_for_anchor(&rtc_tick);
	if (ret != 0) {
		LOG_ERR("MAX30001 ECG SAMP anchor failed: %d", ret);
		goto out;
	}
	anchor_ready = true;
	ecg_next_rtc_tick = rtc_tick;

	/* Disable the MCU edge first, then remove SAMP from the sensor output. */
	ret = gpio_pin_interrupt_configure_dt(&ecg_intb2, GPIO_INT_DISABLE);
	if (ret != 0) {
		LOG_ERR("Failed to disable MAX30001 INT2B interrupt: %d", ret);
		goto out;
	}

	ret = max30001_ecg_disable_samp_interrupt();
	if (ret != 0) {
		LOG_ERR("Failed to mask MAX30001 SAMP interrupt: %d", ret);
		goto out;
	}

	k_sem_give(&ecg_fifo_sem);
	LOG_INF("ECG NAND recording active, first sample RTC tick=%u",
		(unsigned int)ecg_next_rtc_tick);

	while (atomic_get(&ecg_record_requested) != 0) {
		ret = k_sem_take(&ecg_fifo_sem, K_SECONDS(1));
		if (ret == 0) {
			ret = ecg_record_drain_fifo();
			if (ret != 0) {
				goto out;
			}
		}
	}
	ret = 0;

out:
	ecg_record_disable_gpio_interrupts();
	if (anchor_ready && ret == 0 &&
	    atomic_get(&ecg_record_requested) == 0) {
		int drain_ret = ecg_record_drain_fifo();

		if (drain_ret != 0) {
			LOG_ERR("MAX30001 ECG final FIFO drain failed: %d", drain_ret);
			ret = drain_ret;
		}
	}
	if (sensor_configured) {
		stop_ret = max30001_ecg_stop();
		if (ret == 0 && stop_ret != 0) {
			ret = stop_ret;
		}
	}
	atomic_set(&ecg_anchor_state, ECG_RECORD_ANCHOR_IDLE);

	return ret;
}

/**
 * @brief Dedicated recorder thread: supervises ECG recording sessions.
 *
 * Created at boot by K_THREAD_DEFINE and never exits. It blocks on
 * ecg_record_start_sem until ecg_recorder_start() signals a session, then
 * repeatedly invokes ecg_record_run() while ecg_record_requested is set.
 * Around each run it maintains ecg_record_active (so ecg_recorder_stop() can
 * tell whether a session is in flight) and gives ecg_record_stopped_sem so
 * the stopper can synchronize with session teardown.
 *
 * If a session ends while a recording is still requested (i.e. it aborted on
 * error rather than being stopped), the error is logged and the session is
 * retried after a 1-second backoff, making recording self-healing across
 * transient sensor or bus failures.
 *
 * @param arg1 Unused.
 * @param arg2 Unused.
 * @param arg3 Unused.
 */
static void ecg_record_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for (;;) {
		(void)k_sem_take(&ecg_record_start_sem, K_FOREVER);

		while (atomic_get(&ecg_record_requested) != 0) {
			int ret;

			atomic_set(&ecg_record_active, 1);
			ret = ecg_record_run();
			atomic_clear(&ecg_record_active);
			k_sem_give(&ecg_record_stopped_sem);

			if (atomic_get(&ecg_record_requested) == 0) {
				break;
			}

			LOG_ERR("ECG recording stopped after error: %d", ret);
			k_sleep(K_SECONDS(1));
		}
	}
}

/**
 * @brief Request that ECG recording begin (public API).
 *
 * Non-blocking entry point called from application code (e.g. when the user
 * enters ECG collection mode). After confirming the filesystem is ready, it
 * atomically transitions ecg_record_requested from 0 to 1 and wakes the
 * recorder thread, which performs all the actual sensor setup and streaming.
 * The compare-and-set makes the call idempotent: if a recording is already
 * requested, nothing happens and 0 is returned.
 *
 * @retval 0 if recording was started or was already running.
 * @retval -ENODEV if the filesystem is not ready.
 */
int ecg_recorder_start(void)
{
	if (!file_system_ready) {
		return -ENODEV;
	}

	k_sem_reset(&ecg_record_stopped_sem);

	if (atomic_cas(&ecg_record_requested, 0, 1)) {
		k_sem_give(&ecg_record_start_sem);
	}

	return 0;
}

/**
 * @brief Request that ECG recording stop and wait for it to finish
 *        (public API).
 *
 * Clears ecg_record_requested so the recorder thread's session loop exits,
 * and gives ecg_fifo_sem to wake the thread immediately rather than letting
 * it wait out its 1-second semaphore timeout. If no session was active the
 * call returns at once; otherwise it blocks (up to 3 seconds) on
 * ecg_record_stopped_sem until the thread has drained the final samples and
 * powered down the sensor, so callers know storage is quiescent when this
 * returns.
 *
 * @retval 0 once recording has stopped (or none was active).
 * @retval -EAGAIN if the active session did not confirm shutdown within 3 s.
 */
int ecg_recorder_stop(void)
{
	atomic_clear(&ecg_record_requested);
	k_sem_give(&ecg_anchor_sem);
	k_sem_give(&ecg_fifo_sem);

	if (atomic_get(&ecg_record_active) == 0) {
		return 0;
	}

	return k_sem_take(&ecg_record_stopped_sem, K_SECONDS(3));
}
