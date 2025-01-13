
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <nrfx_qspi.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/rand32.h>
#include <time.h>


#include <stdlib.h>
#include "zephyrfilesystem.h"


LOG_MODULE_REGISTER(zephyrfilesystem, 3);

#if CONFIG_DISK_DRIVER_FLASH
#include <zephyr/storage/flash_map.h>
#endif

#if CONFIG_DISK_DRIVER_RAW_NAND
#include "drivers/nand/spi_nand.h"
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



#define MAX_BUFFER_SIZE 9000

#define GET_FATTIME() (DWORD)get_current_unix_time()
#undef GET_FATTIME()



// Might need to put this and the timer in a seperate file.

struct k_work_q my_work_q;

memory_container ppg_work_item;

memory_container accel_work_item;

//data limit per file in bytes
static int data_limit = MAX_BUFFER_SIZE;




// external globals
int storage_percent_full;

int upload_timeout_errors;

bool file_lock;

uint64_t last_time_update_sent;

uint64_t set_date_time = 0;

int patient_num = 0;

int packet_number = 0;

int last_packet_number_processed = 0;

static int current_file_count;

const int max_writes = 256;

typedef struct k_sensor_upload {
	enum sensor_type sensor;
	struct k_work work;
};


typedef struct data_upload_buffer {
	char data_upload_buffer[MAX_BUFFER_SIZE];
	size_t current_size;
} data_upload_buffer;


// settings
bool use_random_files = false;
bool direct_write_file = true; 




// internally linked globals
static struct fs_mount_t fs_mnt;
//counter to serve as a amount for when the file fills up.
static int data_counter;
char file_name[50] = "";
static bool first_write = false;
static struct fs_file_t file;



typedef struct MotionSenseFile {
	int write_size;
	int current_writes;
	int data_counter;
	char sensor_string[5];
	char file_name[50];
	char sensor_format[80];
	struct fs_file_t self_file;
	bool switch_buffer;
	data_upload_buffer buffer1;
	data_upload_buffer buffer2;
	
} 	MotionSenseFile;



//File Objects
static MotionSenseFile current_file;

MotionSenseFile ppg_file = {
	.write_size = 8192,
	.sensor_string = "ppg",
	.sensor_format = "4 channels of uint32 ppg and uint32 global counter"
};

MotionSenseFile accel_file = {
	.write_size = 8192,
	.sensor_string = "ac",
	.sensor_format = "3 int16 accel, 3 float32 gyro, uint32 global counter"
};


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


void create_test_file(int sectors){
	printk("trying to write file...\n");
	
	char destination[50] = "";
	int ID = 0;
	struct fs_mount_t* mp = &fs_mnt;
	char IDString[5];


	struct fs_file_t test_file;
	fs_file_t_init(&test_file);
	
	ID = sys_rand32_get() % 90000;
	itoa(ID, IDString,  10);

	strcat(destination, mp->mnt_point);
	strcat(destination, "/");
	strcat(destination, IDString);
	strcat(destination, "testing.txt");
	int file_create = fs_open(&test_file, destination, FS_O_CREATE | FS_O_WRITE);
	if (file_create == 0)
	{
		char a[4096 * 2] = "hello world";
		for (int i = 0; i < sectors; i++)
		{
			printk("trying to write...\n");
			fs_write(&test_file, a, sizeof(a));
		}
		printk("done writing\n");
		fs_close(&test_file);
	}
}

void create_test_files(int number_of_files){

	for (int x = 0; x < number_of_files; x++){
		create_test_file(256);
	}


}

void create_sensor_file(MotionSenseFile* MSenseFile){

}

void reset_sensor_file(MotionSenseFile* MSenseFile){
	fs_close(&MSenseFile->self_file);
	MSenseFile->buffer1.current_size = 0;
	MSenseFile->buffer2.current_size = 0;
	MSenseFile->switch_buffer = false;
	MSenseFile->current_writes = 0;
}

void sensor_write_to_file(const void* data, size_t size, enum sensor_type sensor){
	struct fs_mount_t* mp = &fs_mnt;
	MotionSenseFile* MSenseFile;
	if (storage_percent_full >= 99){
		LOG_WRN("Storage is getting full, aborting write");
		return;
	}
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == accelorometer){
		MSenseFile = &accel_file;
	}


	if (MSenseFile->current_writes == 0){
		// Create a new file, with given sensor type, patient name, and date
		fs_file_t_init(&MSenseFile->self_file);
		
		
		int ID = 0;
		char IDString[5];
		char patient_id[6];
		if (use_random_files){
			
		
			ID = sys_rand32_get() % 900;
			
		}
		else {
			uint64_t current_time = get_current_unix_time();
			ID = current_time;

		}
		itoa(ID, IDString,  10);
		

		memset(MSenseFile->file_name, 0, sizeof(MSenseFile->file_name));
		strcat(MSenseFile->file_name, mp->mnt_point);
		strcat(MSenseFile->file_name, "/");
		if (patient_num != 0){
			itoa(patient_num, patient_id, 10);
			strcat(MSenseFile->file_name, patient_id);	
		}
		strcat(MSenseFile->file_name, MSenseFile->sensor_string);
		strcat(MSenseFile->file_name, IDString);
		strcat(MSenseFile->file_name, ".bin");
		//printk("file: %s \n", file_name); 
		int file_create = fs_open(&MSenseFile->self_file, MSenseFile->file_name, FS_O_CREATE | FS_O_WRITE);
		if (file_create != 0){
			LOG_WRN("Unable to create file");
		}
		// we write in sizes of 4096*2, so we include that in the formula
		FRESULT res = f_expand(MSenseFile->self_file.filep, 4096*max_writes*2, 1);
		if (res != 0){
		LOG_WRN("failed to expand file");
		}
	}
	else if (data_counter >= data_limit){
		//memset(file_name, 0, sizeof(file_name));
		data_counter = 0;
	}
	
	int total_written = fs_write(&MSenseFile->self_file, data, size);
	MSenseFile->current_writes++;
	//fs_write(&file, data, size);
	if (total_written == size){
		LOG_INF("sucessfully wrote to file, bytes written = %i ! \n", total_written);
		data_counter += total_written;
	}
	if (MSenseFile->current_writes >= max_writes){
		fs_close(&MSenseFile->self_file);
		LOG_INF("closing file\n");
		MSenseFile->current_writes = 0;
		get_storage_percent_full();
	}
}

// writes data to a single file named 'test.txt' future TODO: make an extra string parameter so that the file name is customizable
void write_to_file(const void* data, size_t size){
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
		itoa(ID, IDString,  10);

		

		strcat(file_name, mp->mnt_point);
		strcat(file_name, "/");
		strcat(file_name, IDString);
		strcat(file_name, "test.txt");
		//printk("file: %s \n", file_name); 
		int file_create = fs_open(&file, file_name, FS_O_CREATE | FS_O_WRITE);
		first_write = true;

	}
	else if (data_counter >= data_limit){
		//memset(file_name, 0, sizeof(file_name));
		data_counter = 0;
	}
	
	int total_written = fs_write(&file, data, size);
	//fs_write(&file, data, size);
	if (total_written = size){
		//printk("sucessfully wrote file, bytes written = %i ! \n", total_written);
		data_counter += total_written;
	}
}




void work_write(struct k_work* item){
	
	memory_container* container =
        CONTAINER_OF(item, memory_container, work);
	start_timer();
	sensor_write_to_file(container->address, container->size, container->sensor);
	stop_timer();
	// packets should always be in FIFO order for the queue, for sake of the data order. This check makes sure this is always ensured.
	if (container->packet_num <= last_packet_number_processed){
		LOG_ERR("FIFO in k_work not met.");	
	}
	LOG_INF("Processing packet %i", container->packet_num);
	last_packet_number_processed = container->packet_num;

}

void submit_write(const void* data, size_t size, enum sensor_type type){
	
	//memcpy(work_item.address, data, size);
	memory_container* work_item;
	if (type == ppg){
		work_item = &ppg_work_item;
	}
	else if (type == accelorometer){
		work_item == &accel_work_item;
	}

	work_item->address = data;
	work_item->size = size;
	work_item->sensor = type;
	packet_number++;
	work_item->packet_num = packet_number;
	int ret = k_work_submit_to_queue(&my_work_q, &work_item->work);
	if (ret != 1){
		upload_timeout_errors += 1;
		LOG_ERR("bad ret value: %i, total_errors: %d", ret, upload_timeout_errors);
		
	}
	LOG_INF("ret value: %i", ret);

}


void store_data(const void* data, size_t size, enum sensor_type sensor){
	LOG_DBG("Store data called");
	data_upload_buffer* current_buffer;
	//int16_t arr[6];
	MotionSenseFile* MSenseFile;
	if (sensor == ppg){
		MSenseFile = &ppg_file;
	}
	else if (sensor == accelorometer){
		MSenseFile = &accel_file;
	}

	if (MSenseFile->switch_buffer){
		current_buffer = &MSenseFile->buffer2;
	}
	else {
		current_buffer = &MSenseFile->buffer1;
	}

	void* address_to_write = &current_buffer->data_upload_buffer[current_buffer->current_size];
	void* result = memcpy(address_to_write, data, size);
	//memcpy(arr, address_to_write, size);
	current_buffer->current_size += size;
	if (current_buffer->current_size + size >= MSenseFile->write_size){
		if (current_buffer->current_size + size != MSenseFile->write_size){
			LOG_WRN("Warning: size of total buffer is overflowing from last, truncating...");
		}
		LOG_INF("Submitting File!");
		submit_write(current_buffer->data_upload_buffer, current_buffer->current_size, sensor);
		current_buffer->current_size = 0;
		MSenseFile->switch_buffer = !MSenseFile->switch_buffer;
	}
}

int write_ble_uuid(const char* uuid){

	struct fs_mount_t* mp = &fs_mnt;
	struct fs_file_t name_file;
	fs_file_t_init(&name_file);
	int res;
	// theoretically zephyr docs say this allows us to test whether the file exists or not?
	char uuid_name[20] = "";
	strcat(uuid_name, mp->mnt_point);
	strcat(uuid_name, "/");
	strcat(uuid_name, "uuid.txt");
	int file_create = fs_open(&name_file, uuid_name, 0);
	// the above function will return error -2 if file name is not present, so we can use it to check whether it gets included or not.
	if (file_create != 0)
	{
		int file_create = fs_open(&name_file, uuid_name, FS_O_CREATE | FS_O_WRITE);
		if (file_create != 0)
		{
			LOG_WRN("Unable to create file");
			return -1;
		}
		// we write in sizes of 4096*2, so we include that in the formula
		// max_writes

		strcat(uuid, "\n ppg format: ");
  		strcat(uuid, ppg_file.sensor_format);

		strcat(uuid, "\n accel format: ");
  		strcat(uuid, accel_file.sensor_format);
		strcat(uuid, "\n for a more complete description of how this device works, please visit https://github.com/SenSE-Lab-OSU/MotionSenseHRV4Flash for more info.");
		res = f_expand(name_file.filep, 4096 * 4, 1);
		res = fs_write(&name_file, uuid, strlen(uuid));
		res = 1;
	}
	else
	{
		res = 0;
	}
	fs_close(&name_file);
	
	return res;
}

int close_all_files(){

	int code = fs_close(&file);
	reset_sensor_file(&accel_file);
	reset_sensor_file(&ppg_file);
	
	return code;

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
	}

	(void)fs_closedir(&dir);

	return;
}


int get_storage_percent_full(){
	struct fs_statvfs info;
	struct fs_mount_t* mp = &fs_mnt;
	int rc = fs_statvfs(mp->mnt_point, &info);
	if (rc < 0) {
		printk("FAIL: statvfs: %d\n", rc);
		return;
	}

	printk("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu\n",
	       mp->mnt_point,
	       info.f_bsize, info.f_frsize,
	       info.f_blocks, info.f_bfree);

	float storage_percent = (info.f_blocks - info.f_bfree);
	storage_percent /= info.f_blocks;
	storage_percent *= 100;
	storage_percent_full = (int)storage_percent;
	LOG_INF("storage: %f and %i and total_errors %i", storage_percent, storage_percent_full, upload_timeout_errors);
	return (int)storage_percent;

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
#define TIMEZONE_SHIFT -7
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

