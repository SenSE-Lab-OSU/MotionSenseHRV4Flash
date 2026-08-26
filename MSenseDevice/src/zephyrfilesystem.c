
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <nrfx_qspi.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <time.h>
#include <stdio.h>
#include <string.h>


#include <stdlib.h>
#include "BLEService.h"
#include "msense_git_metadata.h"
#include "zephyrfilesystem.h"


LOG_MODULE_REGISTER(zephyrfilesystem, 3);

#if CONFIG_DISK_DRIVER_FLASH
#include <zephyr/storage/flash_map.h>
#endif

#if CONFIG_DISK_DRIVER_RAW_NAND
#include "drivers/nand/spi_nand.h"
#include "drivers/nand/nand_disk.h"
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

memory_container ecg_work_item;

memory_container log_work_item;

//data limit per file in bytes
const int data_limit = MAX_BUFFER_SIZE;




// external globals
uint8_t storage_percent_full;

int upload_timeout_errors;

bool reset_lock;

uint64_t last_time_update_sent;

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
static bool collection_id_valid;
static uint64_t active_collection_id;
//counter to serve as a amount for when the file fills up.
static int data_counter;
char file_name[50] = "";
static bool first_write = false;
static struct fs_file_t file;



typedef struct MotionSenseFile {
	int write_size;
	int max_writes;
	int current_writes;
	uint32_t chunk_index;
	int data_counter;
	uint64_t start_time;
	bool first_sample_init;
	const char sensor_string[5];
	char file_name[96];
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
	.max_writes = 512,
	.sensor_string = "ppg",
	.sensor_format = "4 channels of uint32 ppg (2 IR then 2 green), uint32 global_tick_512hz"
};

/* store_data checks one sample ahead; this flushes 8184-byte ECG chunks. */
MotionSenseFile ecg_file = {
	.write_size = 8196,
	/*
	 * 512 8,196-byte writes would extend a 4 MiB preallocated file by
	 * 2 KiB.  Keep the file's allocated logical size fixed instead.
	 */
	.max_writes = RECORDING_FILE_BYTES / 8196,
	.sensor_string = "ecg",
	.sensor_format = "12-byte MAX30001 ECG frames: A5 EC type flags rtc_tick_le raw24 crc8"
};

MotionSenseFile log_file = {
	.write_size = 8192,
	.max_writes = 512,
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
        case ecg: return "ecg";
        case customlog: return "log";
        default:           return "undefined";
    }
}

void filesystem_set_collection_id(uint64_t collection_id)
{
	active_collection_id = collection_id;
	collection_id_valid = true;
	ecg_file.chunk_index = 0U;
}

void filesystem_clear_collection_id(void)
{
	active_collection_id = 0U;
	collection_id_valid = false;
}

int filesystem_make_recording_path(char *path, size_t path_size,
				   const char *stream_prefix,
				   uint64_t collection_id)
{
	return filesystem_make_recording_chunk_path(path, path_size, stream_prefix,
						    collection_id, 0U);
}

int filesystem_make_recording_chunk_path(char *path, size_t path_size,
					 const char *stream_prefix,
					 uint64_t collection_id,
					 uint32_t chunk_index)
{
	int written;

	if ((path == NULL) || (path_size == 0U) || (stream_prefix == NULL) ||
	    !file_system_ready) {
		return -EINVAL;
	}

	if (patient_num != 0) {
		written = snprintf(path, path_size, "%s/%d%s%llu_%04lu.bin",
				   fs_mnt.mnt_point, patient_num, stream_prefix,
				   (unsigned long long)collection_id,
				   (unsigned long)chunk_index);
	} else {
		written = snprintf(path, path_size, "%s/%s%llu_%04lu.bin",
				   fs_mnt.mnt_point, stream_prefix,
				   (unsigned long long)collection_id,
				   (unsigned long)chunk_index);
	}

	if ((written < 0) || ((size_t)written >= path_size)) {
		return -ENAMETOOLONG;
	}

	return 0;
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

	
void create_test_file(int writes){
	printk("write file...\n");
	
	char destination[50] = "";
	int ID = 0;
	struct fs_mount_t* mp = &fs_mnt;
	char IDString[12];


	struct fs_file_t test_file;
	fs_file_t_init(&test_file);
	total_test_files++;
	//ID = sys_rand32_get() % 90000;
	ID = total_test_files;
	
	snprintf(IDString, sizeof(IDString), "%d", ID);

	strcat(destination, mp->mnt_point);
	strcat(destination, "/");
	strcat(destination, IDString);
	strcat(destination, "testing.txt");
	int file_create = fs_open(&test_file, destination, FS_O_CREATE | FS_O_WRITE);
	FIL* fatfs_file_ptr = (FIL *)test_file.filep;
	FRESULT res = f_expand(fatfs_file_ptr, 4096*max_writes*2, 1);
	if (res != 0){
		LOG_WRN("failed to expand test file");
	}
	if (file_create == 0)
	{
		
		for (int i = 0; i < writes; i++)
		{
			//k_sleep(K_SECONDS(.5)); for debug purposes in case it's ever necessary to see how files write slowly
			fs_write(&test_file, test_file_arr, sizeof(test_file_arr));
		}
		printk("done write\n");
		fs_close(&test_file);
	}
}

void create_test_files(int number_of_files){
	LOG_INF("creating test files...");
	for (int x = 0; x < number_of_files; x++){
		LOG_INF("file %d of %d", x, number_of_files);
		create_test_file(512);
	}


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

void reset_sensor_file(MotionSenseFile* MSenseFile){
	fs_close(&MSenseFile->self_file);
	MSenseFile->buffer1.current_size = 0;
	MSenseFile->buffer2.current_size = 0;
	MSenseFile->switch_buffer = false;
	MSenseFile->current_writes = 0;
	MSenseFile->first_sample_init = false;
}




void shutdown_filesystem(){
	close_all_files();
	file_system_ready = false;
	//struct fs_mount_t* mp = &fs_mnt;
	//fs_unmount(mp);
}

void reset_log_file(){
	reset_sensor_file(&log_file);
}

void sensor_write_to_file(const void* data, size_t size, enum sensor_type sensor){
	struct fs_mount_t* mp = &fs_mnt;
	MotionSenseFile* MSenseFile;
	if (storage_percent_full >= 99){
		LOG_WRN("Storage is getting full, aborting write");
		return;
	}

	if (IS_ENABLED(CONFIG_DISK_DRIVER_RAW_NAND) && get_read_only()){
		LOG_WRN("Tried to write while in readonly mode, aborting");
		return;
	}
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == ecg){
		MSenseFile = &ecg_file;
	}
	else if (sensor == customlog){
		MSenseFile = &log_file;
	}
	else {
		LOG_WRN("invalid file type given");
		return;
	}


	if (MSenseFile->current_writes == 0){
		// Create a new file, with given sensor type, patient name, and date as file name
		fs_file_t_init(&MSenseFile->self_file);

		if ((sensor == ecg) && collection_id_valid) {
			int path_ret = filesystem_make_recording_chunk_path(
				MSenseFile->file_name, sizeof(MSenseFile->file_name),
				MSenseFile->sensor_string, active_collection_id,
				MSenseFile->chunk_index);

			if (path_ret != 0) {
				LOG_WRN("Unable to construct ECG recording path: %d", path_ret);
				file_system_malfunction = true;
				return;
			}
		} else {
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


			memset(MSenseFile->file_name, 0, sizeof(MSenseFile->file_name));
			strcat(MSenseFile->file_name, mp->mnt_point);
			strcat(MSenseFile->file_name, "/");
			if (patient_num != 0){
				snprintf(patient_id, sizeof(patient_id), "%d", patient_num);
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
		}

		// Now that we created the file name, open it and write the data
		LOG_INF("Creating new file for %d", sensor);
		if ((sensor == ecg) && collection_id_valid) {
			struct fs_dirent entry;
			int stat_ret = fs_stat(MSenseFile->file_name, &entry);

			if (stat_ret == 0) {
				LOG_WRN("ECG recording chunk already exists");
				file_system_malfunction = true;
				return;
			}
			if (stat_ret != -ENOENT) {
				LOG_WRN("Unable to check ECG recording chunk: %d", stat_ret);
				file_system_malfunction = true;
				return;
			}
		}
		int file_create = fs_open(&MSenseFile->self_file, MSenseFile->file_name, FS_O_CREATE | FS_O_WRITE);
		if (file_create != 0){
			LOG_WRN("Unable to create file for %d", sensor);
			file_system_malfunction = true;
			return;
		}
		// we write in sizes of 4096*2, so we include that in the formula
		FRESULT res = f_expand(MSenseFile->self_file.filep,
					       RECORDING_FILE_BYTES, 1);
		if (res != 0){
			LOG_WRN("failed to expand file");
			file_system_malfunction = true;
			(void)fs_close(&MSenseFile->self_file);
			return;
		}
	}
	else if (data_counter >= data_limit){
		data_counter = 0;
	}
	
	int total_written = fs_write(&MSenseFile->self_file, data, size);
	MSenseFile->current_writes++;
	//fs_write(&file, data, size);
	if (total_written == size){
		LOG_INF("successfully wrote to file for %d, bytes = %i, writes = %i !", sensor, total_written, MSenseFile->current_writes);
		file_system_malfunction = false;
		data_counter += total_written;
	}
	else if (total_written < 0){
		LOG_WRN("system failed %d write for %d! tot writes before: %d ", size, sensor, MSenseFile->current_writes);
		file_system_malfunction = true;
		status_reg_ble_notification();
	}

	if (MSenseFile->current_writes >= MSenseFile->max_writes){
		// if we don't want the leftover empty sectors caused by the buffer writes being smaller than 8192 size we can
		// uncomment these lines or use f_truncate() with dhara
		//FIL* fp = &MSenseFile->self_file.filep;
		//fp->obj.objsize = fp->fptr;
		//fp->flag |= 0x40; // = FA_MODIFIED
		int close_ret = fs_close(&MSenseFile->self_file);
		LOG_INF("closing file\n");
		if (close_ret < 0){
			file_system_malfunction = 5;
			status_reg_ble_notification();
			LOG_WRN("Error on closing file");
		}
		MSenseFile->current_writes = 0;
		if ((sensor == ecg) && collection_id_valid) {
			MSenseFile->chunk_index++;
		}
		get_storage_percent_full();
	}
}

// writes data to a single file named 'test.txt' future TODO: make an extra string parameter so that the file name is customizable
int write_to_file(const void* data, size_t size){
	struct fs_mount_t* mp = &fs_mnt;
	if (!first_write ){
		
		fs_file_t_init(&file);
		
		
		int ID = 0;
		char IDString[5];
		if (use_random_files){
			
		
			ID = sys_rand32_get() % 900;
			
		}
		else {
			uint64_t current_time = get_current_unix_time();
			ID = current_time;

		}
		sprintf(IDString, "%d", ID);

		

		strcat(file_name, mp->mnt_point);
		strcat(file_name, "/");
		strcat(file_name, IDString);
		strcat(file_name, "test.txt");
		//printk("file: %s \n", file_name); 
		int file_create = fs_open(&file, file_name, FS_O_CREATE | FS_O_WRITE);
		if (file_create != 0){
			return file_create;
		}
		first_write = true;

	}
	else if (data_counter >= data_limit){
		//memset(file_name, 0, sizeof(file_name));
		data_counter = 0;
	}
	
	int total_written = fs_write(&file, data, size);
	//fs_write(&file, data, size);
	if (total_written == size){
		//printk("sucessfully wrote file, bytes written = %i ! \n", total_written);
		data_counter += total_written;
		return 0;
	}
	return -1;

}




void work_write(struct k_work* item){
	
	memory_container* container =
        CONTAINER_OF(item, memory_container, work);
	LOG_INF("Processing packet %i", container->packet_num);
	start_timer();
	LOG_DBG("writing true for container %d", container->sensor);
	container->in_use = true;
	sensor_write_to_file(container->address, container->size, container->sensor);
	int64_t time_value = stop_timer();
	LOG_INF("write timer: %lli", time_value);
	// packets should always be in FIFO order for the queue, for sake of the data order. This check makes sure this is always ensured.
	if (container->packet_num <= last_packet_number_processed){
		LOG_ERR("FIFO in k_work not met.");	
	}
	
	last_packet_number_processed = container->packet_num;
	LOG_DBG("writing false for container %d", container->sensor);
	container->in_use = false;

}

void submit_write(const void* data, size_t size, enum sensor_type type){
	
	//memcpy(work_item.address, data, size);
	memory_container* work_item;
	if (type == ppg){
		work_item = &ppg_work_item;
	}
	else if (type == ecg){
		work_item = &ecg_work_item;
	}
	else if (type == customlog){
		work_item = &log_work_item;
	}
	else {
		LOG_WRN("invalid file type given");
		return;
	}
	int work_status = k_work_busy_get(&work_item->work);
	if (work_status != 0){
		LOG_WRN("work state for %d not zero", type);
	}
	LOG_DBG("state for sensor %d: %d", type, work_status);
	
	if (!work_item->in_use){

	work_item->address = data;
	work_item->size = size;
	work_item->sensor = type;
	packet_number++;
	work_item->packet_num = packet_number;
	int ret = k_work_submit_to_queue(&my_work_q, &work_item->work);
	if (ret != 1){
		upload_timeout_errors += 1;
		LOG_ERR("bad ret value for sensor %i: %i, total_errors: %d", type, ret, upload_timeout_errors);
		
	}
	LOG_INF("ret value for %i: %i", type, ret);
	}
	else{
		LOG_ERR("work item attempted schedule while still running for type: %i", type);
	}
}


void store_data(const void* data, size_t size, enum sensor_type sensor){
	LOG_DBG("Store data called");
	data_upload_buffer* current_buffer;
	//int16_t arr[6];
	MotionSenseFile* MSenseFile;
	bool first_init = false;
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == ecg){
		MSenseFile = &ecg_file;
	}
	else if (sensor == customlog){
		MSenseFile = &log_file;
	}
	else{
		LOG_WRN("sensor type unknown");
		return;
	}

	if (MSenseFile->switch_buffer){
		current_buffer = &MSenseFile->buffer2;
	}
	else {
		current_buffer = &MSenseFile->buffer1;
	}

	if (!MSenseFile->first_sample_init){
		if (sensor == ecg && collection_id_valid) {
			MSenseFile->start_time = active_collection_id;
		} else {
			MSenseFile->start_time = get_current_unix_time();
		}
		MSenseFile->first_sample_init = true;
		first_init = true;
	}

	void* address_to_write = &current_buffer->data_upload_buffer[current_buffer->current_size];
	memcpy(address_to_write, data, size);
	current_buffer->current_size += size;
	if (current_buffer->current_size + size >= MSenseFile->write_size){
		if (current_buffer->current_size + size != MSenseFile->write_size){
			LOG_WRN("Wrn: tot size is %d short. this is ok but will cause few 0xff at EOF.", MSenseFile->write_size - current_buffer->current_size);
		}
		if ((MSenseFile->current_writes + 1) >= MSenseFile->max_writes){
			MSenseFile->first_sample_init = false;
		}
		if (!panic_single_thread){
		LOG_INF("Submitting Write!");
		submit_write(current_buffer->data_upload_buffer, current_buffer->current_size, sensor);
		}
		else {
			sensor_write_to_file(current_buffer->data_upload_buffer, current_buffer->current_size, sensor);
		}
		current_buffer->current_size = 0;
		MSenseFile->switch_buffer = !MSenseFile->switch_buffer;
	}
}

void flush_data_buffer(enum sensor_type sensor){
	
	LOG_DBG("Flush data called");
	data_upload_buffer* current_buffer;
	//int16_t arr[6];
	MotionSenseFile* MSenseFile;
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == ecg){
		MSenseFile = &ecg_file;
	}
	else if (sensor == customlog){
		MSenseFile = &log_file;
	}
	else{
		LOG_WRN("sensor type unknown");
		return;
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
			if ((MSenseFile->current_writes + 1) >= MSenseFile->max_writes){
				MSenseFile->first_sample_init = false;
			}
			if (!panic_single_thread){
			LOG_INF("Submitting Write!");
			submit_write(current_buffer->data_upload_buffer, current_buffer->current_size, sensor);
			}
			else {
				sensor_write_to_file(current_buffer->data_upload_buffer, current_buffer->current_size, sensor);
			}
			current_buffer->current_size = 0;
			MSenseFile->switch_buffer = !MSenseFile->switch_buffer;
	}
	else {
		LOG_INF("empty buffers, no flushing required");
	}
}


int write_ble_uuid(const char *ble_address, const char *ble_name,
			   const char *device_id_hex)
{
	struct fs_mount_t *mp = &fs_mnt;
	struct fs_file_t name_file;
	char uuid_name[32];
	char uuid_contents[UUID_CONTENTS_MAX_SIZE];
	int rc;
	int written;
	ssize_t bytes_written;

	if (ble_address == NULL || ble_name == NULL || device_id_hex == NULL) {
		return -EINVAL;
	}

	written = snprintf(uuid_name, sizeof(uuid_name), "%s/uuid.txt", mp->mnt_point);
	if (written < 0 || written >= sizeof(uuid_name)) {
		return -ENAMETOOLONG;
	}

	fs_file_t_init(&name_file);
	rc = fs_open(&name_file, uuid_name, FS_O_READ);
	if (rc == 0) {
		fs_close(&name_file);
		return 0;
	}
	if (rc != -ENOENT) {
		LOG_WRN("Unable to check uuid.txt: %d", rc);
		return rc;
	}

	written = snprintf(uuid_contents, sizeof(uuid_contents),
			   "%s\nName: %s\nDevice ID: %s\nVersion: %s"
			   "\nGit Commit: %s\nGit Tree: %s"
			   "\nppg format: %s\naccel format: ICM-20948 accel binary format v2"
			   "\necg format: %s"
			   "\nFor a more complete description of how this device works, please visit "
			   "https://github.com/SenSE-Lab-OSU/MotionSenseHRV4Flash for more info.\n",
			   ble_address, ble_name, device_id_hex, CONFIG_BT_DIS_MODEL,
			   MSENSE_GIT_COMMIT, MSENSE_GIT_TREE_STATE,
			   ppg_file.sensor_format, ecg_file.sensor_format);
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
		rc = 1;
	}

	{
		int close_rc = fs_close(&name_file);

		if (rc > 0 && close_rc != 0) {
			rc = close_rc;
		}
	}

	return rc;
}

int close_all_files(){

	
	reset_sensor_file(&ppg_file);
	reset_sensor_file(&ecg_file);
	reset_sensor_file(&log_file);
	return 0;

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

void setup_disk(void)
{
	struct fs_mount_t *mp = &fs_mnt;
	struct fs_dir_t dir;
	struct fs_statvfs sbuf;
	int rc;
	
	fs_dir_t_init(&dir);

	if (IS_ENABLED(CONFIG_DISK_DRIVER_FLASH)) {
		rc = setup_flash(mp);
		if (rc < 0) {
			LOG_ERR("Failed to setup flash area");
			return;
		}
	}

	if (!IS_ENABLED(CONFIG_FILE_SYSTEM_LITTLEFS) &&
	    !IS_ENABLED(CONFIG_FAT_FILESYSTEM_ELM)) {
		LOG_INF("No file system selected");
		return;
	}

	rc = mount_app_fs(mp);
	if (rc < 0) {
		LOG_ERR("Failed to mount filesystem");
		return;
	}
	/* Allow log messages to flush to avoid interleaved output */
	k_sleep(K_MSEC(50));

	LOG_INF("Mount %s: %d\n", fs_mnt.mnt_point, rc);

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		printk("FAIL: statvfs: %d\n", rc);
		return;
	}

	LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu\n",
	       mp->mnt_point,
	       sbuf.f_bsize, sbuf.f_frsize,
	       sbuf.f_blocks, sbuf.f_bfree);

	rc = fs_opendir(&dir, mp->mnt_point);
	LOG_INF("%s opendir: %d\n", mp->mnt_point, rc);

	if (rc < 0) {
		LOG_ERR("Failed to open directory");
	}

	while (rc >= 0) {
		struct fs_dirent ent = { 0 };

		rc = fs_readdir(&dir, &ent);
		if (rc < 0) {
			LOG_ERR("Failed to read directory entries");
			break;
		}
		if (ent.name[0] == 0) {
			LOG_INF("End of files\n");
			break;
		}
		LOG_INF("  %c %u %s\n",
		       (ent.type == FS_DIR_ENTRY_FILE) ? 'F' : 'D',
		       ent.size,
		       ent.name);
		
		strstr(ent.name, "test") != NULL ? total_test_files++ : 0;
		strstr(ent.name, "log") != NULL ? total_log_files++ : 0;
		

	}
	file_system_ready = true;
	(void)fs_closedir(&dir);

	return;
}


int get_storage_percent_full(){
	struct fs_statvfs info;
	struct fs_mount_t* mp = &fs_mnt;
	int rc = fs_statvfs(mp->mnt_point, &info);
	if (rc < 0) {
		printk("FAIL: statvfs: %d\n", rc);
		return -1;
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




#include "drivers/nand/nand_disk.h"
// The following shows how to use the nand disk driver outside of the driver file directly.


#define DT_DRV_COMPAT senselab_nanddisk
int test_desk_driver(){
	uint8_t write_buf[4096] = {1};
	uint8_t read_buf[4096];
	// you can do:
	//const struct device* filesystem_device = DEVICE_DT_INST_GET(0);
	// OR
	const struct device* filesystem_device2 = sdmmc_disk.dev;
	spi_nand_page_write(filesystem_device2, 63, write_buf, sizeof(read_buf));
	spi_nand_page_write(filesystem_device2, 64, write_buf, sizeof(read_buf));
	spi_nand_page_read(filesystem_device2, 63, read_buf);
	print_page_hex(read_buf, sizeof(read_buf), true);
	spi_nand_block_erase(filesystem_device2, 0);
	spi_nand_page_read(filesystem_device2, 63, read_buf);
	print_page_hex(read_buf, sizeof(read_buf), true);
	spi_nand_page_read(filesystem_device2, 64, read_buf);
	print_page_hex(read_buf, sizeof(read_buf), true);
	return 0;
}



int read_storage_percent_full(){
	return storage_percent_full;
}


void set_date_time_bt(uint64_t value){
	
	set_date_time = value;
	last_time_update_sent = k_uptime_get() / 1000;
	LOG_INF("new datetime sent, value is %llu, seconds uptime is %llu", set_date_time, last_time_update_sent);
}

uint64_t get_current_unix_time(){
	
	uint64_t current_upime = k_uptime_get();
	current_upime /= 1000;
	LOG_DBG("current uptime in seconds: %llu", current_upime);
	uint64_t current_time = (current_upime - last_time_update_sent) + set_date_time;
	LOG_DBG("current timestamp: %llu", current_time);
	return current_time;
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

void start_timer(){
	start_time = k_uptime_get();
}


int64_t stop_timer(){
	int64_t length = k_uptime_get() - start_time;
	start_time = 0;
	LOG_INF("Timer Value: %lli ms", length);
	return length;
}
