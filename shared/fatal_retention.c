#include <zephyr/arch/arm/cortex_m/exception.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <pm_config.h>
#include <stddef.h>
#include <stdint.h>

#include "msense_fatal_retention.h"
#include "msense_git_metadata.h"

LOG_MODULE_REGISTER(crash, LOG_LEVEL_INF);

#define RETENTION_NODE DT_NODELABEL(msense_fatal_retention)
#define RETENTION_SIZE DT_REG_SIZE(RETENTION_NODE)
#define RECORD_MAGIC 0x4d534652U
#define COUNTER_MAGIC 0x434e5452U
#define RECORD_VERSION 1U
#define FIRMWARE_ID_SIZE 12U

#define RECORD_FLAG_ARM_SNAPSHOT BIT(0)
#define RECORD_FLAG_FRAME_VALID BIT(1)

#define STACKING_ERROR_MASK (SCB_CFSR_MSTKERR_Msk | SCB_CFSR_MUNSTKERR_Msk | \
	SCB_CFSR_STKERR_Msk | SCB_CFSR_UNSTKERR_Msk | SCB_CFSR_MLSPERR_Msk | \
	SCB_CFSR_LSPERR_Msk | SCB_CFSR_STKOF_Msk)

struct retained_fatal_record {
	uint32_t magic;
	uint32_t counter_magic;
	uint32_t count;
	uint32_t count_inverse;
	uint16_t version;
	uint16_t size;
	uint32_t checksum;
	uint32_t reason;
	uint32_t stage;
	uint32_t flags;
	uint32_t thread;
	uint32_t fault_vector;
	uint32_t msp;
	uint32_t psp;
	uint32_t exc_return;
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t xpsr;
	uint32_t cfsr;
	uint32_t hfsr;
	uint32_t dfsr;
	uint32_t afsr;
	uint32_t mmfar;
	uint32_t bfar;
	char firmware_id[FIRMWARE_ID_SIZE];
};

struct arm_fault_snapshot {
	bool valid;
	uint32_t fault_vector;
	uint32_t msp;
	uint32_t psp;
	uint32_t exc_return;
	uint32_t cfsr;
	uint32_t hfsr;
	uint32_t dfsr;
	uint32_t afsr;
	uint32_t mmfar;
	uint32_t bfar;
};

static volatile struct retained_fatal_record retained_record
	__attribute__((section(LINKER_DT_NODE_REGION_NAME(RETENTION_NODE)), used));
static struct arm_fault_snapshot arm_snapshot;
static volatile enum msense_fatal_stage current_stage;
static bool capture_active;
static const char firmware_id[] = MSENSE_GIT_COMMIT;

BUILD_ASSERT(DT_NODE_HAS_STATUS(RETENTION_NODE, okay));
BUILD_ASSERT(RETENTION_SIZE >= 256U);
BUILD_ASSERT(sizeof(retained_record) <= RETENTION_SIZE);
BUILD_ASSERT(PM_MSENSE_FATAL_RETENTION_ADDRESS == DT_REG_ADDR(RETENTION_NODE));
BUILD_ASSERT(PM_MSENSE_FATAL_RETENTION_SIZE == RETENTION_SIZE);

static uint32_t record_checksum(const volatile struct retained_fatal_record *record)
{
	const volatile uint8_t *bytes = (const volatile uint8_t *)&record->reason;
	const size_t length = sizeof(*record) - offsetof(struct retained_fatal_record, reason);
	uint32_t hash = 2166136261U;

	for (size_t i = 0; i < length; ++i) {
		hash = (hash ^ bytes[i]) * 16777619U;
	}
	return hash;
}

static bool record_valid(void)
{
	return retained_record.magic == RECORD_MAGIC &&
	       retained_record.counter_magic == COUNTER_MAGIC &&
	       retained_record.count_inverse == ~retained_record.count &&
	       retained_record.version == RECORD_VERSION &&
	       retained_record.size == sizeof(retained_record) &&
	       retained_record.checksum == record_checksum(&retained_record);
}

void msense_fatal_stage_set(enum msense_fatal_stage stage)
{
	current_stage = stage;
}

void z_arm_fault_retention_snapshot(uint32_t fault_vector, uint32_t msp,
				    uint32_t psp, uint32_t exc_return,
				    uint32_t cfsr, uint32_t hfsr,
				    uint32_t dfsr, uint32_t afsr,
				    uint32_t mmfar, uint32_t bfar)
{
	arm_snapshot.fault_vector = fault_vector;
	arm_snapshot.msp = msp;
	arm_snapshot.psp = psp;
	arm_snapshot.exc_return = exc_return;
	arm_snapshot.cfsr = cfsr;
	arm_snapshot.hfsr = hfsr;
	arm_snapshot.dfsr = dfsr;
	arm_snapshot.afsr = afsr;
	arm_snapshot.mmfar = mmfar;
	arm_snapshot.bfar = bfar;
	barrier_dmem_fence_full();
	arm_snapshot.valid = true;
}

void z_arm_fault_retention_discard(void)
{
	arm_snapshot.valid = false;
}

void z_fatal_error_retention_capture(unsigned int reason,
				     const struct arch_esf *esf,
				     struct k_thread *thread)
{
	uint32_t count;
	bool frame_valid;

	if (capture_active) {
		return;
	}
	capture_active = true;
	count = retained_record.counter_magic == COUNTER_MAGIC &&
		retained_record.count_inverse == ~retained_record.count ?
		retained_record.count + 1U : 1U;
	frame_valid = esf != NULL &&
		(!arm_snapshot.valid || !(arm_snapshot.cfsr & STACKING_ERROR_MASK));

	retained_record.magic = 0U;
	barrier_dmem_fence_full();
	retained_record.counter_magic = 0U;
	retained_record.count = count;
	retained_record.count_inverse = ~count;
	barrier_dmem_fence_full();
	retained_record.counter_magic = COUNTER_MAGIC;
	retained_record.version = RECORD_VERSION;
	retained_record.size = sizeof(retained_record);
	retained_record.reason = reason;
	retained_record.stage = current_stage;
	retained_record.flags = (arm_snapshot.valid ? RECORD_FLAG_ARM_SNAPSHOT : 0U) |
		(frame_valid ? RECORD_FLAG_FRAME_VALID : 0U);
	retained_record.thread = (uint32_t)(uintptr_t)thread;
	retained_record.fault_vector = arm_snapshot.valid ? arm_snapshot.fault_vector : 0U;
	retained_record.msp = arm_snapshot.valid ? arm_snapshot.msp : 0U;
	retained_record.psp = arm_snapshot.valid ? arm_snapshot.psp : 0U;
	retained_record.exc_return = arm_snapshot.valid ? arm_snapshot.exc_return : 0U;
	retained_record.cfsr = arm_snapshot.valid ? arm_snapshot.cfsr : 0U;
	retained_record.hfsr = arm_snapshot.valid ? arm_snapshot.hfsr : 0U;
	retained_record.dfsr = arm_snapshot.valid ? arm_snapshot.dfsr : 0U;
	retained_record.afsr = arm_snapshot.valid ? arm_snapshot.afsr : 0U;
	retained_record.mmfar = arm_snapshot.valid ? arm_snapshot.mmfar : 0U;
	retained_record.bfar = arm_snapshot.valid ? arm_snapshot.bfar : 0U;
	retained_record.r0 = frame_valid ? esf->basic.r0 : 0U;
	retained_record.r1 = frame_valid ? esf->basic.r1 : 0U;
	retained_record.r2 = frame_valid ? esf->basic.r2 : 0U;
	retained_record.r3 = frame_valid ? esf->basic.r3 : 0U;
	retained_record.r12 = frame_valid ? esf->basic.r12 : 0U;
	retained_record.lr = frame_valid ? esf->basic.lr : 0U;
	retained_record.pc = frame_valid ? esf->basic.pc : 0U;
	retained_record.xpsr = frame_valid ? esf->basic.xpsr : 0U;
	for (size_t i = 0; i < FIRMWARE_ID_SIZE; ++i) {
		retained_record.firmware_id[i] =
			i < sizeof(firmware_id) - 1U ? firmware_id[i] : '-';
	}
	retained_record.checksum = record_checksum(&retained_record);
	barrier_dmem_fence_full();
	retained_record.magic = RECORD_MAGIC;
	barrier_dmem_fence_full();
	arm_snapshot.valid = false;
}

void msense_fatal_retention_report(void)
{
	char reported_firmware_id[FIRMWARE_ID_SIZE + 1U];

	if (!record_valid()) {
		return;
	}
	for (size_t i = 0; i < FIRMWARE_ID_SIZE; ++i) {
		reported_firmware_id[i] = retained_record.firmware_id[i];
	}
	reported_firmware_id[FIRMWARE_ID_SIZE] = '\0';

	LOG_ERR("Retained fatal record follows");
	LOG_ERR("magic=%08x counter_magic=%08x", retained_record.magic,
		retained_record.counter_magic);
	LOG_ERR("version=%u size=%u checksum=%08x", retained_record.version,
		retained_record.size, retained_record.checksum);
	LOG_ERR("count=%u count_inverse=%08x", retained_record.count,
		retained_record.count_inverse);
	LOG_ERR("firmware=%s", reported_firmware_id);
	LOG_ERR("reason=%u stage=%u flags=%08x", retained_record.reason,
		retained_record.stage, retained_record.flags);
	LOG_ERR("thread=%08x vector=%u", retained_record.thread,
		retained_record.fault_vector);
	LOG_ERR("msp=%08x psp=%08x", retained_record.msp, retained_record.psp);
	LOG_ERR("exc_return=%08x", retained_record.exc_return);
	LOG_ERR("r0=%08x r1=%08x", retained_record.r0, retained_record.r1);
	LOG_ERR("r2=%08x r3=%08x", retained_record.r2, retained_record.r3);
	LOG_ERR("r12=%08x lr=%08x", retained_record.r12, retained_record.lr);
	LOG_ERR("pc=%08x xpsr=%08x", retained_record.pc, retained_record.xpsr);
	LOG_ERR("cfsr=%08x hfsr=%08x", retained_record.cfsr,
		retained_record.hfsr);
	LOG_ERR("dfsr=%08x afsr=%08x", retained_record.dfsr,
		retained_record.afsr);
	LOG_ERR("mmfar=%08x bfar=%08x", retained_record.mmfar,
		retained_record.bfar);
}

FUNC_NORETURN void msense_normal_reset(void)
{
	volatile uint32_t *words = (volatile uint32_t *)DT_REG_ADDR(RETENTION_NODE);

	for (size_t i = 0; i < RETENTION_SIZE / sizeof(*words); ++i) {
		words[i] = 0U;
	}
	barrier_dmem_fence_full();
	NVIC_SystemReset();
	CODE_UNREACHABLE;
}
