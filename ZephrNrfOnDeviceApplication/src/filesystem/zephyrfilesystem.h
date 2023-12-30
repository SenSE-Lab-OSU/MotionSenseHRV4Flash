


enum sensor_type {ppg, 
accelorometer, passthrough};


void setup_disk(void);

void create_test_files();

void sensor_write_to_file(const void* data, size_t size, enum sensor_type);

void write_to_file(const void* data, size_t size);

int close_all_files();

void submit_write(const void* data, size_t size, enum sensor_type type);


void store_data(const void* data, size_t size, enum sensor_type sensor);