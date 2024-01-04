


enum sensor_type {ppg, 
accelorometer, passthrough};

typedef struct memory_container {
	char address[50];
	size_t size;
	enum sensor_type sensor;
	int packet_num;
	struct k_work work;

} memory_container;

extern struct k_work_q my_work_q;

void setup_disk(void);

void create_test_files();

void sensor_write_to_file(const void* data, size_t size, enum sensor_type);

void write_to_file(const void* data, size_t size);

int close_all_files();

void submit_write(const void* data, size_t size, enum sensor_type type);


void store_data(const void* data, size_t size, enum sensor_type sensor);

int get_storage_percent_full();

extern int storage_percent_full;

//k work item
void work_write(struct k_work* item);

uint64_t get_current_unix_time();

void set_date_time_bt(uint64_t value);

void start_timer();

int64_t stop_timer();

extern int64_t start_time;

extern memory_container work_item;

