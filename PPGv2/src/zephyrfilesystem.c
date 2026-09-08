
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <nrfx_qspi.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <errno.h>
#include <stdio.h>
#include <time.h>


#include <stdlib.h>
#include "BLEService.h"
#include "msense_git_metadata.h"
#include "zephyrfilesystem.h"


LOG_MODULE_REGISTER(zephyrfilesystem, 3);

#if CONFIG_DISK_DRIVER_FLASH
#include <zephyr/storage/flash_map.h>
#endif

#if CONFIG_DISK_DRIVER_RAW_NAND
#include "spi_nand.h"
#include "nand_disk.h"
#endif

#if CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(PM_LITTLEFS_STORAGE_NAME)
#endif

#if CONFIG_FILE_SYSTEM_LITTLEFS
#include <zephyr/fs/littlefs.h>
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);

#define STORAGE_PARTITION		storage_partition
#define STORAGE_PARTITION_ID		FIXED_PARTITION_ID(STORAGE_PARTITION)
#endif

bool file_system_ready;

bool security_lock;

bool panic_single_thread;

#define MAX_BUFFER_SIZE 9000
#define UUID_CONTENTS_MAX_SIZE 640U

//#undef GET_FATTIME
//#define GET_FATTIME() (DWORD)get_current_unix_time()



// Might need to put this and the timer in a seperate file.

struct k_work_q my_work_q;

memory_container ppg_work_item;

memory_container accel_work_item;

memory_container log_work_item;

//data limit per file in bytes
const int data_limit = MAX_BUFFER_SIZE;




// external globals
uint8_t storage_percent_full;

int upload_timeout_errors;

bool reset_lock;

uint64_t last_time_update_uptime_ms;

uint64_t set_date_time = 0;

int patient_num = 0;

int packet_number = 0;

int last_packet_number_processed = 0;


const int max_writes = 512;



typedef struct data_upload_buffer {
	char data_upload_buffer[MAX_BUFFER_SIZE];
	size_t current_size;
} data_upload_buffer;


// settings
bool use_random_files = false;
bool direct_write_file = true; 




// internally linked globals
static struct fs_mount_t fs_mnt;
static bool filesystem_mounted;
//counter to serve as a amount for when the file fills up.
static int data_counter;
static bool first_write = false;
static struct fs_file_t file;
static int close_all_files(void);



typedef struct MotionSenseFile {
	int write_size;
	int current_writes;
	int data_counter;
	uint64_t start_time;
	bool first_sample_init;
	const char sensor_string[5];
	char file_name[50];
	const char sensor_format[90];
	struct fs_file_t self_file;
	bool switch_buffer;
	data_upload_buffer buffer1;
	data_upload_buffer buffer2;
	
} 	MotionSenseFile;



//File Objects
//static MotionSenseFile current_file;

MotionSenseFile ppg_file = {
	.write_size = 8192,
	.sensor_string = "ppg",
	.sensor_format = "uint24_le ir1, uint24_le ir2, uint24_le g1, uint24_le g2, uint32_le global_tick_512hz"
};

MotionSenseFile accel_file = {
	.write_size = 8192,
	.sensor_string = "ac",
	.sensor_format = "3 int16 accel, 3 float32 quaternion, second avg float32 enmo, uint32 global_tick_512hz"
};

MotionSenseFile log_file = {
	.write_size = 8192,
	.sensor_string = "log",
	.sensor_format = "logging"
};

// TODO: Still work in progress . We do have a hacky way to make the device read only (see nand_disk.c) 
// but no way to make it show up as read only on windows yet.
void enable_read_only(bool enable){
	struct fs_mount_t* mp = &fs_mnt;
	if (mp->type == FS_FATFS){
		#if CONFIG_FAT_FILESYSTEM_ELM
		if (enable){
			//f_chmod(mp->mnt_point, AM_RDO, AM_RDO);
		}
		else{
			//f_chmod(mp->mnt_point, 0, AM_RDO);
		}
		#endif
	}
}

const char* sensor_enum_to_string(enum sensor_type sensor) {
    switch (sensor) {
        case ppg:    return "ppg";
        case accelorometer:  return "acc";
        case customlog: return "log";
        default:           return "undefined";
    }
}


int total_test_files = 0;
int total_log_files = 0;







// Test files
char test_file_arr[4096*2] = "hello world, this is a story about a man who liked to run. "
    "every day for miles. he wandered and wandered for miles. "
    "as the seasons changed, he kept moving, tracing the edges of towns "
    "and forests, learning the quiet language of the wind. "
    "people sometimes asked him why he ran so far, but he only smiled, "
    "because the answer was something he felt rather than spoke. "
    "the rhythm of his footsteps steadied his thoughts, and the long roads "
    "gave him room to remember who he was and who he hoped to become. "
    "on certain mornings, when the fog clung low to the fields, he felt "
    "as though he were the only person awake in the world. "
    "he liked those mornings best. "
    "they reminded him that solitude was not the same as loneliness; "
    "it was a kind of quiet companionship with the earth itself. "
    "and so he kept running, mile after mile, year after year, "
    "carrying stories in his breath and dreams in his stride.";

	
int create_test_file(int writes)
{
	char destination[50] = "";
	char id_string[12];
	struct fs_file_t test_file;
	struct fs_mount_t *mp = &fs_mnt;
	FRESULT expand_ret;
	int close_ret;
	int ret;
	int sync_ret;
	ssize_t write_ret;

	if (!file_system_ready || !filesystem_mounted) {
		LOG_ERR("Filesystem is unavailable for test-file creation");
		return -EACCES;
	}
	if (writes < 0) {
		return -EINVAL;
	}

	printk("write file...\n");
	fs_file_t_init(&test_file);
	total_test_files++;
	ret = snprintf(id_string, sizeof(id_string), "%d", total_test_files);
	if (ret < 0 || ret >= (int)sizeof(id_string)) {
		return -ENAMETOOLONG;
	}

	strcat(destination, mp->mnt_point);
	strcat(destination, "/");
	strcat(destination, id_string);
	strcat(destination, "testing.txt");
	ret = fs_open(&test_file, destination, FS_O_CREATE | FS_O_WRITE);
	if (ret != 0) {
		return ret;
	}

	expand_ret = f_expand((FIL *)test_file.filep, 4096 * max_writes * 2, 1);
	if (expand_ret != FR_OK) {
		LOG_WRN("Failed to expand test file: %d", expand_ret);
		ret = -EIO;
		goto close_file;
	}

	ret = 0;
	for (int i = 0; i < writes; i++) {
		write_ret = fs_write(&test_file, test_file_arr, sizeof(test_file_arr));
		if (write_ret < 0) {
			ret = (int)write_ret;
			break;
		}
		if (write_ret != (ssize_t)sizeof(test_file_arr)) {
			ret = -EIO;
			break;
		}
	}

close_file:
	sync_ret = fs_sync(&test_file);
	close_ret = fs_close(&test_file);
	if (sync_ret != 0) {
		LOG_ERR("Test-file sync failed: %d", sync_ret);
	}
	if (close_ret != 0) {
		LOG_ERR("Test-file close failed: %d", close_ret);
	}
	if (ret == 0 && sync_ret != 0) {
		ret = sync_ret;
	}
	if (ret == 0 && close_ret != 0) {
		ret = close_ret;
	}
	if (ret == 0) {
		printk("done write\n");
	}

	return ret;
}

int create_test_files(int number_of_files)
{
	int ret;

	if (number_of_files < 0) {
		return -EINVAL;
	}

	LOG_INF("creating test files...");
	for (int x = 0; x < number_of_files; x++) {
		LOG_INF("file %d of %d", x, number_of_files);
		ret = create_test_file(512);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

bool is_file_open(struct fs_file_t *zfp) {
    // If filep is not NULL, the file wrapper structure is currently active/open
    return (zfp != NULL && zfp->filep != NULL);
}

bool file_exists(const char *path)
{
    struct fs_dirent entry;
    
    // fs_stat returns 0 on success (file exists)
    int ret = fs_stat(path, &entry);
    
    if (ret == 0) {
        // Optional: Ensure it's a file and not a directory
        return entry.type == FS_DIR_ENTRY_FILE;
    }
    
    // Returns -ENOENT if the file does not exist
    return false;
}

bool filesystem_is_mounted(void)
{
	return filesystem_mounted;
}

int filesystem_drain_pending_work(void)
{
	int ret;

	ret = k_work_queue_drain(&my_work_q, false);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

int filesystem_gate_and_drain(void)
{
	int ret;

	ret = k_work_queue_drain(&my_work_q, true);
	file_system_ready = false;
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int sync_and_close_file(struct fs_file_t *file_to_close)
{
	int sync_ret;
	int close_ret;

	if (file_to_close->mp == NULL) {
		return 0;
	}

	sync_ret = fs_sync(file_to_close);
	close_ret = fs_close(file_to_close);
	if (sync_ret != 0) {
		return sync_ret;
	}

	return close_ret;
}

static int reset_sensor_file(MotionSenseFile *msense_file)
{
	int ret;

	ret = sync_and_close_file(&msense_file->self_file);
	if (ret != 0) {
		return ret;
	}

	fs_file_t_init(&msense_file->self_file);
	msense_file->buffer1.current_size = 0;
	msense_file->buffer2.current_size = 0;
	msense_file->switch_buffer = false;
	msense_file->current_writes = 0;
	msense_file->first_sample_init = false;
	return 0;
}




int shutdown_filesystem(void)
{
	int close_ret;
	int unmount_ret;

	file_system_ready = false;
	if (!filesystem_mounted) {
		return 0;
	}

	close_ret = close_all_files();
	unmount_ret = fs_unmount(&fs_mnt);
	if (unmount_ret == 0) {
		filesystem_mounted = false;
	} else {
		LOG_ERR("Failed to unmount filesystem: %d", unmount_ret);
	}

	if (close_ret != 0) {
		return close_ret;
	}

	return unmount_ret;
}

void reset_log_file(){
	LOG_ERR("Direct log-file reset is disabled outside the PPG storage owner");
	ppg_collection_latch_storage_fault();
}

static int sensor_write_failure(enum sensor_type sensor, const char *operation,
				int error)
{
	int ret = error < 0 ? error : -EIO;

	file_system_malfunction = true;
	status_reg_ble_notification();
	LOG_WRN("%s failed for sensor %d: %d", operation, sensor, error);
	return ret;
}

static int sensor_write_to_file(const void* data, size_t size, enum sensor_type sensor){
	struct fs_mount_t* mp = &fs_mnt;
	MotionSenseFile* MSenseFile;
	int close_ret;
	int storage_ret;
	int sync_ret;
	ssize_t total_written;
	FRESULT expand_ret;

	if (!file_system_ready || !filesystem_mounted) {
		return sensor_write_failure(sensor, "Filesystem unavailable", -EACCES);
	}
	if (storage_percent_full >= 99){
		return sensor_write_failure(sensor, "Storage full", -ENOSPC);
	}

	if (IS_ENABLED(CONFIG_DISK_DRIVER_RAW_NAND) && get_read_only()){
		return sensor_write_failure(sensor, "Raw disk is read-only", -EROFS);
	}
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == accelorometer){
		MSenseFile = &accel_file;
	}
	else if (sensor == customlog){
		MSenseFile = &log_file;
	}
	else {
		return sensor_write_failure(sensor, "Invalid sensor type", -EINVAL);
	}


	if (MSenseFile->current_writes == 0){
		// Create a new file, with given sensor type, patient name, and date as file name
		fs_file_t_init(&MSenseFile->self_file);
		
		
		uint64_t ID = 0;
		// max itoa can do is 33 with binary, but theoretically it will be < 9
		char IDString[33];
		char patient_id[33];
		if (use_random_files){
			
		
			ID = sys_rand32_get() % 900;
			
		}
		else {

			uint64_t current_time = MSenseFile->start_time; 
			
			ID = current_time;
			if (sensor == customlog){
				// could also add it onto the time instead?
				total_log_files++;
				ID = total_log_files;
			}

		}
		sprintf(IDString, "%llu", ID);
		//itoa(ID, IDString,  10);
		

		memset(MSenseFile->file_name, 0, sizeof(MSenseFile->file_name));
		strcat(MSenseFile->file_name, mp->mnt_point);
		strcat(MSenseFile->file_name, "/");
		if (patient_num != 0){
			itoa(patient_num, patient_id, 10);
			strcat(MSenseFile->file_name, patient_id);	
		}
		strcat(MSenseFile->file_name, MSenseFile->sensor_string);
		strcat(MSenseFile->file_name, IDString);
		if (sensor != customlog){
			strcat(MSenseFile->file_name, ".bin");
		}
		else {
			strcat(MSenseFile->file_name, ".txt");
		}
		
		// Now that we created the file name, open it and write the data
		LOG_INF("Creating new file for %d", sensor);
		int file_create = fs_open(&MSenseFile->self_file, MSenseFile->file_name, FS_O_CREATE | FS_O_WRITE);
		if (file_create != 0){
			return sensor_write_failure(sensor, "File open", file_create);
		}
		if (MSenseFile->self_file.filep == NULL) {
			close_ret = fs_close(&MSenseFile->self_file);
			if (close_ret != 0) {
				LOG_WRN("File close after invalid open state failed for sensor %d: %d",
					sensor, close_ret);
			}
			return sensor_write_failure(sensor, "File open returned no file handle",
						    -EIO);
		}
		// we write in sizes of 4096*2, so we include that in the formula
		expand_ret = f_expand(MSenseFile->self_file.filep,
					      4096 * max_writes * 2, 1);
		if (expand_ret != FR_OK){
			close_ret = fs_close(&MSenseFile->self_file);
			if (close_ret != 0) {
				LOG_WRN("File close after expansion failure failed for sensor %d: %d",
					sensor, close_ret);
			}
			return sensor_write_failure(sensor, "File expansion", -EIO);
		}
	}
	else if (data_counter >= data_limit){
		data_counter = 0;
	}
	
	total_written = fs_write(&MSenseFile->self_file, data, size);
	if (total_written == (ssize_t)size){
		MSenseFile->current_writes++;
		LOG_DBG("sucessfully wrote to file for %d, bytes = %i, writes = %i ! \n", sensor, total_written, MSenseFile->current_writes);
		file_system_malfunction = false;
		data_counter += total_written;
	}
	else if (total_written < 0){
		return sensor_write_failure(sensor, "File write", (int)total_written);
	}
	else {
		return sensor_write_failure(sensor, "Short file write", -EIO);
	}

	if (MSenseFile->current_writes >= max_writes){
		// if we don't want the leftover empty sectors caused by the buffer writes being smaller than 8192 size we can
		// uncomment these lines or use f_truncate() with dhara
		//FIL* fp = &MSenseFile->self_file.filep;
		//fp->obj.objsize = fp->fptr;
		//fp->flag |= 0x40; // = FA_MODIFIED
		sync_ret = fs_sync(&MSenseFile->self_file);
		close_ret = fs_close(&MSenseFile->self_file);
		LOG_INF("closing file\n");
		if (sync_ret != 0) {
			return sensor_write_failure(sensor, "File rollover sync", sync_ret);
		}
		if (close_ret != 0){
			return sensor_write_failure(sensor, "File rollover close", close_ret);
		}
		MSenseFile->current_writes = 0;
		storage_ret = get_storage_percent_full();
		if (storage_ret < 0) {
			return sensor_write_failure(sensor, "Storage capacity query", storage_ret);
		}
	}

	return 0;
}

int write_to_file(const void* data, size_t size)
{
	ARG_UNUSED(data);
	ARG_UNUSED(size);
	LOG_ERR("Direct filesystem writes are disabled outside the PPG storage owner");
	ppg_collection_latch_storage_fault();
	return -ENOTSUP;
}


int64_t file_system_timer;

void work_write(struct k_work* item){
	
	memory_container* container =
        CONTAINER_OF(item, memory_container, work);
	int write_ret;

	LOG_DBG("Processing packet %i", container->packet_num);
	start_timer(&file_system_timer);
	LOG_DBG("writing true for container %d", container->sensor);
	container->in_use = true;
	write_ret = sensor_write_to_file(container->address, container->size,
					 container->sensor);
	int64_t time_value = stop_timer(&file_system_timer);
	LOG_DBG("write timer: %lli", time_value);
	// packets should always be in FIFO order for the queue, for sake of the data order. This check makes sure this is always ensured.
	if (container->packet_num <= last_packet_number_processed){
		LOG_ERR("FIFO in k_work not met.");	
	}
	
	last_packet_number_processed = container->packet_num;
	LOG_DBG("writing false for container %d", container->sensor);
	container->in_use = false;
	if (write_ret != 0) {
		LOG_ERR("Filesystem write failed for sensor %d: %d", container->sensor,
			write_ret);
		ppg_collection_latch_storage_fault();
	}

}

int submit_write(const void* data, size_t size, enum sensor_type type){
	
	//memcpy(work_item.address, data, size);
	memory_container* work_item;
	int ret;

	if (!file_system_ready || !filesystem_mounted) {
		return -EACCES;
	}

	if (type == ppg){
		work_item = &ppg_work_item;
	}
	else if (type == accelorometer){
		work_item = &accel_work_item;
	}
	else if (type == customlog){
		work_item = &log_work_item;
	}
	else {
		LOG_WRN("invalid file type given");
		return -EINVAL;
	}
	int work_status = k_work_busy_get(&work_item->work);
	if (work_status != 0){
		LOG_WRN("work state for %d not zero", type);
		return -EBUSY;
	}
	LOG_DBG("state for sensor %d: %d", type, work_status);
	
	if (work_item->in_use){
		LOG_ERR("work item attempted schedule while still running for type: %i", type);
		return -EBUSY;
	}

	work_item->address = data;
	work_item->size = size;
	work_item->sensor = type;
	packet_number++;
	work_item->packet_num = packet_number;
	ret = k_work_submit_to_queue(&my_work_q, &work_item->work);
	if (ret != 1){
		upload_timeout_errors += 1;
		LOG_ERR("bad ret value for sensor %i: %i, total_errors: %d", type, ret, upload_timeout_errors);
		return (ret < 0) ? ret : -EALREADY;
	}
	LOG_DBG("ret value for %i: %i", type, ret);
	return 0;
}


int store_data(const void* data, size_t size, enum sensor_type sensor){
	LOG_DBG("Store data called");
	data_upload_buffer* current_buffer;
	//int16_t arr[6];
	MotionSenseFile* MSenseFile;
	int ret;
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == accelorometer){
		MSenseFile = &accel_file;
	}
	else if (sensor == customlog){
		MSenseFile = &log_file;
	}
	else{
		LOG_WRN("sensor type unknown");
		ppg_collection_latch_storage_fault();
		return -EINVAL;
	}

	if (MSenseFile->switch_buffer){
		current_buffer = &MSenseFile->buffer2;
	}
	else {
		current_buffer = &MSenseFile->buffer1;
	}
	if (current_buffer->current_size >= MSenseFile->write_size) {
		LOG_ERR("Completed buffer for %d is still awaiting ownership", sensor);
		ppg_collection_latch_storage_fault();
		return -EBUSY;
	}
	if (size > sizeof(current_buffer->data_upload_buffer) -
		    current_buffer->current_size) {
		LOG_ERR("Buffer capacity exceeded for %d", sensor);
		ppg_collection_latch_storage_fault();
		return -ENOSPC;
	}

	if (!MSenseFile->first_sample_init){
		MSenseFile->start_time = get_current_unix_time();
		MSenseFile->first_sample_init = true;
	}

	void* address_to_write = &current_buffer->data_upload_buffer[current_buffer->current_size];
	memcpy(address_to_write, data, size);
	current_buffer->current_size += size;
	if (current_buffer->current_size >= MSenseFile->write_size){
		if (current_buffer->current_size != MSenseFile->write_size){
			LOG_WRN("Wrn: tot size is %d short. this is ok but will cause few 0xff at EOF.", MSenseFile->write_size - current_buffer->current_size);
		}
		if (panic_single_thread) {
			LOG_ERR("Cannot verify a direct buffer write during panic mode");
			ppg_collection_latch_storage_fault();
			return -ENOTSUP;
		}
		LOG_DBG("Submitting Write!");
		ret = submit_write(current_buffer->data_upload_buffer,
					   current_buffer->current_size, sensor);
		if (ret != 0) {
			LOG_ERR("Unable to submit completed buffer for %d: %d", sensor, ret);
			ppg_collection_latch_storage_fault();
			return ret;
		}
		if ((MSenseFile->current_writes + 1) >= max_writes){
			MSenseFile->first_sample_init = false;
		}
		/* The worker now owns this buffer; only then may this stream switch. */
		current_buffer->current_size = 0;
		MSenseFile->switch_buffer = !MSenseFile->switch_buffer;
	}

	return 0;
}

int flush_data_buffer(enum sensor_type sensor){
	
	LOG_DBG("Flush data called");
	data_upload_buffer* current_buffer;
	//int16_t arr[6];
	MotionSenseFile* MSenseFile;
	int ret;
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == accelorometer){
		MSenseFile = &accel_file;
	}
	else if (sensor == customlog){
		MSenseFile = &log_file;
	}
	else{
		LOG_WRN("sensor type unknown");
		return -EINVAL;
	}

	if (MSenseFile->switch_buffer){
		current_buffer = &MSenseFile->buffer2;
	}
	else {
		current_buffer = &MSenseFile->buffer1;
	}
	if (current_buffer->current_size != 0){
		if (current_buffer->current_size != MSenseFile->write_size){
				LOG_WRN("Wrn: tot size is %d short. this is ok but will cause few 0xff at EOF.", MSenseFile->write_size - current_buffer->current_size);
			}
			if (panic_single_thread){
				LOG_ERR("Cannot verify a direct flush during panic mode");
				return -ENOTSUP;
			}
			LOG_DBG("Submitting Write!");
			ret = submit_write(current_buffer->data_upload_buffer,
					   current_buffer->current_size, sensor);
			if (ret != 0) {
				LOG_ERR("Unable to submit final buffer for %d: %d", sensor, ret);
				return ret;
			}
			if ((MSenseFile->current_writes + 1) >= max_writes){
				MSenseFile->first_sample_init = false;
			}
			current_buffer->current_size = 0;
			MSenseFile->switch_buffer = !MSenseFile->switch_buffer;
	}
	else {
		LOG_INF("empty buffers, no flushing required");
	}

	return 0;
}


int write_device_info_file(const char *device_name,
				   const char *device_id_hex, const char *dis_model)
{
	struct fs_mount_t *mp = &fs_mnt;
	struct fs_file_t name_file;
	char uuid_name[32];
	char uuid_contents[UUID_CONTENTS_MAX_SIZE];
	int rc;
	int written;
	ssize_t bytes_written;

	if (device_name == NULL || device_id_hex == NULL || dis_model == NULL) {
		return -EINVAL;
	}
	if (!file_system_ready || !filesystem_mounted) {
		return -EACCES;
	}

	written = snprintf(uuid_name, sizeof(uuid_name), "%s/uuid.txt", mp->mnt_point);
	if (written < 0 || written >= sizeof(uuid_name)) {
		return -ENAMETOOLONG;
	}

	fs_file_t_init(&name_file);
	rc = fs_open(&name_file, uuid_name, FS_O_READ);
	if (rc == 0) {
		return fs_close(&name_file);
	}
	if (rc != -ENOENT) {
		LOG_WRN("Unable to check uuid.txt: %d", rc);
		return rc;
	}

	written = snprintf(uuid_contents, sizeof(uuid_contents),
			   "Name: %s\nDevice ID: %s\nVersion: %s"
			   "\nGit Commit: %s\nGit Tree: %s"
			   "\nppg format: %s\naccel format: %s"
			   "\nFor a more complete description of how this device works, please visit "
			   "https://github.com/SenSE-Lab-OSU/MotionSenseHRV4Flash for more info.\n",
			   device_name, device_id_hex, dis_model,
			   MSENSE_GIT_COMMIT, MSENSE_GIT_TREE_STATE,
			   ppg_file.sensor_format, accel_file.sensor_format);
	if (written < 0 || written >= sizeof(uuid_contents)) {
		return -ENOSPC;
	}

	rc = fs_open(&name_file, uuid_name, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		LOG_WRN("Unable to create uuid.txt: %d", rc);
		return rc;
	}

	bytes_written = fs_write(&name_file, uuid_contents, written);
	if (bytes_written < 0) {
		rc = (int)bytes_written;
	} else if (bytes_written != written) {
		rc = -EIO;
	} else {
		rc = 0;
	}

	{
		int sync_rc = 0;
		int close_rc;

		if (rc == 0) {
			sync_rc = fs_sync(&name_file);
		}
		close_rc = fs_close(&name_file);
		if (rc == 0 && sync_rc != 0) {
			rc = sync_rc;
		}
		if (rc == 0 && close_rc != 0) {
			rc = close_rc;
		}
	}

	return rc;
}

static int close_all_files(void)
{
	int ret = 0;
	int close_ret;

	close_ret = reset_sensor_file(&accel_file);
	if (ret == 0 && close_ret != 0) {
		ret = close_ret;
	}

	close_ret = reset_sensor_file(&ppg_file);
	if (ret == 0 && close_ret != 0) {
		ret = close_ret;
	}

	close_ret = reset_sensor_file(&log_file);
	if (ret == 0 && close_ret != 0) {
		ret = close_ret;
	}

	close_ret = sync_and_close_file(&file);
	if (ret == 0 && close_ret != 0) {
		ret = close_ret;
	}
	if (close_ret == 0) {
		fs_file_t_init(&file);
		first_write = false;
		data_counter = 0;
	}

	return ret;
}


static int setup_flash(struct fs_mount_t *mnt)
{
	int rc = 0;
#if CONFIG_DISK_DRIVER_FLASH
	unsigned int id;
	const struct flash_area *pfa;

	mnt->storage_dev = (void *)STORAGE_PARTITION_ID;
	id = STORAGE_PARTITION_ID;

	rc = flash_area_open(id, &pfa);
	LOG_INF("Area %u at 0x%x on %s for %u bytes\n",
	       id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
	       (unsigned int)pfa->fa_size);

	if (rc < 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		printk("Erasing flash area ... ");
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		printk("%d\n", rc);
	}

	if (rc < 0) {
		flash_area_close(pfa);
	}
#endif
	return rc;
}

static int mount_app_fs(struct fs_mount_t *mnt)
{
	int rc;

#if CONFIG_FAT_FILESYSTEM_ELM
	static FATFS fat_fs;
	//FS_MOUNT_FLAG_USE_DISK_ACCESS
	//mnt->flags = FS_MOUNT_FLAG_READ_ONLY;
	mnt->type = FS_FATFS;
	mnt->fs_data = &fat_fs;
	if (IS_ENABLED(CONFIG_DISK_DRIVER_RAM)) {
		mnt->mnt_point = "/RAM:";
	} else if (IS_ENABLED(CONFIG_DISK_DRIVER_SDMMC) | IS_ENABLED(CONFIG_DISK_DRIVER_RAW_NAND) | 
	IS_ENABLED(CONFIG_DISK_DRIVER_FLASH)) {
		mnt->mnt_point = "/SD:";
	}

#elif CONFIG_FILE_SYSTEM_LITTLEFS
	mnt->type = FS_LITTLEFS;
	mnt->mnt_point = "/lfs";
	mnt->fs_data = &storage;
#endif
	rc = fs_mount(mnt);

	return rc;
}

int setup_disk(void)
{
	struct fs_mount_t *mp = &fs_mnt;
	struct fs_dir_t dir;
	struct fs_statvfs sbuf;
	bool dir_open = false;
	int close_ret;
	int unmount_ret;
	int rc;

	if (filesystem_mounted) {
		return file_system_ready ? 0 : -EBUSY;
	}

	file_system_ready = false;
	total_test_files = 0;
	total_log_files = 0;
	fs_dir_t_init(&dir);

	if (IS_ENABLED(CONFIG_DISK_DRIVER_FLASH)) {
		rc = setup_flash(mp);
		if (rc < 0) {
			LOG_ERR("Failed to setup flash area: %d", rc);
			return rc;
		}
	}

	if (!IS_ENABLED(CONFIG_FILE_SYSTEM_LITTLEFS) &&
	    !IS_ENABLED(CONFIG_FAT_FILESYSTEM_ELM)) {
		LOG_ERR("No file system selected");
		return -ENOTSUP;
	}

	rc = mount_app_fs(mp);
	if (rc < 0) {
		LOG_ERR("Failed to mount filesystem: %d", rc);
		return rc;
	}
	filesystem_mounted = true;
	/* Allow log messages to flush to avoid interleaved output. */
	k_sleep(K_MSEC(50));

	LOG_INF("Mount %s: %d", fs_mnt.mnt_point, rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		LOG_ERR("statvfs failed: %d", rc);
		goto unmount;
	}

	LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu",
	       mp->mnt_point,
	       sbuf.f_bsize, sbuf.f_frsize,
	       sbuf.f_blocks, sbuf.f_bfree);

	rc = fs_opendir(&dir, mp->mnt_point);
	LOG_INF("%s opendir: %d", mp->mnt_point, rc);
	if (rc < 0) {
		LOG_ERR("Failed to open directory");
		goto unmount;
	}
	dir_open = true;

	for (;;) {
		struct fs_dirent ent = { 0 };

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			LOG_ERR("Failed to read directory entries");
			break;
		}
		if (ent.name[0] == 0) {
			LOG_INF("End of files");
			break;
		}
		LOG_INF("  %c %u %s",
		       (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
		       ent.size,
		       ent.name);

		if (strstr(ent.name, "test") != NULL) {
			total_test_files++;
		}
		if (strstr(ent.name, "log") != NULL) {
			total_log_files++;
		}
	}

	close_ret = fs_closedir(&dir);
	dir_open = false;
	if (close_ret != 0 && rc == 0) {
		rc = close_ret;
	}
	if (rc != 0) {
		goto unmount;
	}

	file_system_ready = true;
	return 0;

unmount:
	if (dir_open) {
		close_ret = fs_closedir(&dir);
		if (rc == 0 && close_ret != 0) {
			rc = close_ret;
		}
	}

	file_system_ready = false;
	unmount_ret = fs_unmount(mp);
	if (unmount_ret == 0) {
		filesystem_mounted = false;
	} else {
		LOG_ERR("Failed to unmount filesystem after setup error: %d", unmount_ret);
		if (rc == 0) {
			rc = unmount_ret;
		}
	}

	return rc;
}


int get_storage_percent_full(){
	struct fs_statvfs info;
	struct fs_mount_t* mp = &fs_mnt;
	int rc;
	if (!file_system_ready || !filesystem_mounted) {
		return -EACCES;
	}
	rc = fs_statvfs(mp->mnt_point, &info);
	if (rc != 0) {
		printk("FAIL: statvfs: %d\n", rc);
		return rc < 0 ? rc : -EIO;
	}

	printk("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu\n",
	       mp->mnt_point,
	       info.f_bsize, info.f_frsize,
	       info.f_blocks, info.f_bfree);

	float storage_percent = (info.f_blocks - info.f_bfree);
	storage_percent /= info.f_blocks;
	storage_percent *= 100;
	storage_percent_full = (uint8_t)storage_percent;
	if (storage_percent_full >= 99){
		file_system_full = true;
	}
	storage_ble_notification(&storage_percent_full, sizeof(storage_percent_full));
	LOG_INF("storage: %f and %i and total_errors %i", (double)storage_percent, storage_percent_full, upload_timeout_errors);
	return (int)storage_percent;

}




#include "nand_disk.h"
// The following shows how to use the nand disk driver outside of the driver file directly.


#define DT_DRV_COMPAT senselab_nanddisk
int test_desk_driver(){
	LOG_ERR("Raw NAND diagnostic writes are disabled outside the PPG storage owner");
	return -ENOTSUP;
}
uint8_t test_read_buf[4096];
void print_out_page(int page_num){
	
	// can also just change this to disk_read()
	const struct device* filesystem_device2 = sdmmc_disk.dev;
	multi_nand_page_read(filesystem_device2, page_num, test_read_buf);
	//disk_nand_access_read(&sdmmc_disk, test_read_buf, page_num, 1);
	if (page_num > 1500){
		disk_nand_access_read(&sdmmc_disk, test_read_buf, page_num + 1, 1);
		disk_nand_access_read(&sdmmc_disk, test_read_buf, page_num + 2, 1);
	}
	
	//(filesystem_device2, page_num, test_read_buf);
	//LOG_INF("with")
	//spi_nand_page_read(filesystem_device2, page_num, test_read_buf);
	print_page_hex(test_read_buf, sizeof(test_read_buf), false);
}


int read_storage_percent_full(){
	return storage_percent_full;
}


void set_date_time_bt(uint64_t value){
	/*
	 * The datetime characteristic uses Unix milliseconds. Keep the phone's full
	 * precision and anchor it to the millisecond-resolution system uptime.
	 */
	set_date_time = value;
	last_time_update_uptime_ms = k_uptime_get();
	LOG_INF("datetime set to %llu milliseconds, uptime is %llu milliseconds",
		set_date_time, last_time_update_uptime_ms);
}

uint64_t get_current_unix_time_ms(){
	uint64_t current_uptime_ms = k_uptime_get();
	uint64_t current_time_ms =
		(current_uptime_ms - last_time_update_uptime_ms) + set_date_time;
	LOG_DBG("current timestamp in milliseconds: %llu", current_time_ms);
	return current_time_ms;
}

uint64_t get_current_unix_time(){
	return get_current_unix_time_ms() / 1000;
}

// for now we will use Mountain Time (UTC -7)
#define TIMEZONE_SHIFT -8
// To make this work with the file system, you will need to set FF_FS_NORTC to 0 in  zephyr\modules\fatfs (line 82).
DWORD get_fattime(void)
{
	time_t t;
	struct tm *stm;

	t = get_current_unix_time();
	// stm = localtime(&t);
	if (set_date_time != 0)
	{
		stm = gmtime(&t);
		return (DWORD)(stm->tm_year - 80) << 25 |
			   (DWORD)(stm->tm_mon + 1) << 21 |
			   (DWORD)stm->tm_mday << 16 |
			   (DWORD)(stm->tm_hour + TIMEZONE_SHIFT) << 11 |
			   (DWORD)stm->tm_min << 5 |
			   (DWORD)stm->tm_sec >> 1;
	}
	LOG_WRN("date time not set, returning nothing for fatfs date");
	return 0;
}

int64_t start_time;

void start_timer(int64_t* start_time_ref){
	if (start_time_ref != NULL){
		if (*start_time_ref != 0){
			LOG_WRN("timer was executed again before it could finish!");
		}
		*start_time_ref = k_uptime_get();
	}
	else{
		start_time = k_uptime_get();
	}
}


int64_t stop_timer(int64_t* start_time_ref){
	int64_t length;
	if (start_time_ref != NULL){
		length = k_uptime_get() - *start_time_ref;
		*start_time_ref = 0;
		LOG_DBG("Timer Value: %lli ms", length);
	}
	else{
		length = k_uptime_get() - start_time;
		start_time = 0;
		LOG_DBG("Timer Value: %lli ms", length);
	}
	return length;
}
