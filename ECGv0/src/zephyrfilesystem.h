#ifndef ZEPHYR_FILESYSTEM_H_
#define ZEPHYR_FILESYSTEM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#define RECORDING_FILE_BYTES (4U * 1024U * 1024U)

extern bool reset_lock;

extern bool file_system_ready;

extern bool panic_single_thread;


extern bool file_system_ready;

enum sensor_type {ecg, passthrough, customlog};

typedef struct memory_container {
	const void* address;
	size_t size;
	enum sensor_type sensor;
	int packet_num;
	bool in_use;
	struct k_work work;

} memory_container;

extern struct k_work_q my_work_q;



int setup_disk(void);
int shutdown_filesystem(void);
int filesystem_drain_pending_work(void);
int filesystem_gate_and_drain(void);
bool filesystem_is_mounted(void);

int create_test_file(int writes);

int create_test_files(int number_of_files);

int sensor_write_to_file(const void* data, size_t size, enum sensor_type);

int write_to_file(const void* data, size_t size);



int submit_write(const void* data, size_t size, enum sensor_type type);


int store_data(const void* data, size_t size, enum sensor_type sensor);

int flush_data_buffer(enum sensor_type sensor);

void filesystem_set_collection_id(uint64_t collection_id);
void filesystem_clear_collection_id(void);
int filesystem_make_recording_path(char *path, size_t path_size,
				   const char *stream_prefix,
				   uint64_t collection_id);
int filesystem_make_recording_chunk_path(char *path, size_t path_size,
					 const char *stream_prefix,
					 uint64_t collection_id,
					 uint32_t chunk_index);

int get_storage_percent_full();

extern uint8_t storage_percent_full;

int write_device_info_file(const char *device_name,
			   const char *device_id_hex, const char *dis_model);

//k work item
void work_write(struct k_work* item);

uint64_t get_current_unix_time();

void set_date_time_bt(uint64_t value);

void start_timer();

int64_t stop_timer();

void enable_read_only(bool enable);

const char* sensor_enum_to_string(enum sensor_type sensor);

extern bool security_lock;

extern int64_t start_time;

extern int patient_num;

extern uint64_t set_date_time;

extern memory_container ecg_work_item;
extern memory_container log_work_item;

#endif /* ZEPHYR_FILESYSTEM_H_ */
