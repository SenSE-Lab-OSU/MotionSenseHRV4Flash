
extern bool file_lock;

enum sensor_type {ppg, 
accelorometer, passthrough};

typedef struct memory_container {
	const void* address;
	size_t size;
	enum sensor_type sensor;
	int packet_num;
	struct k_work work;

} memory_container;

extern struct k_work_q my_work_q;

void setup_disk(void);

void create_test_file(int sectors);

void create_test_files(int number_of_files);

void sensor_write_to_file(const void* data, size_t size, enum sensor_type);

void write_to_file(const void* data, size_t size);

int close_all_files();

void submit_write(const void* data, size_t size, enum sensor_type type);


void store_data(const void* data, size_t size, enum sensor_type sensor);

int get_storage_percent_full();

extern int storage_percent_full;

int write_ble_uuid(const char* ble_string);

//k work item
void work_write(struct k_work* item);

uint64_t get_current_unix_time();

void set_date_time_bt(uint64_t value);

void start_timer();

int64_t stop_timer();

void enable_read_only(bool enable);

extern int64_t start_time;

extern int patient_num;

extern uint64_t set_date_time;

extern memory_container ppg_work_item;
extern memory_container accel_work_item;

