import os
import struct
import re
def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.



def process_data(data) -> int:

    errors = 0
    check_array = [1,2,3,4,5,6]
    for data_byte in range(0, len(data), 2):
        result = struct.unpack("<h", data[data_byte:data_byte+2])[0]
        #print(result)
        inx = data_byte//2 % 6
        if (result != check_array[inx]):
            #print("error: got " + str(result[0]) + " expected " + str(check_array[data_byte//2 % 6]))
            errors += 1
    print("errors: " + str(errors))
    return errors


def process_data(data, categories: list[list], format: list) -> int:
    # we always assume that the last category is the packet counter
    assert len(categories) == len(format)
    errors = 0
    counter_value = 0
    current_index = 0
    num_of_categories = len(categories)
    for data_byte in range(0, len(data), 2):
        try:
            result = struct.unpack("<h", data[data_byte:data_byte+2])[0]
            #print(result)
            categories[current_index].append(result)
            if current_index == num_of_categories:
                # we are on the last category, which means we are looking at the counter
                current_index = 0
                if (result != counter_value + 1):
                    print("error: got " + str(result) + " expected " + str(counter_value + 1))
                    errors += 1
        except Exception as e:
            errors += 1
            print(e)

    return errors









    print("errors: " + str(errors))
    return errors



def file_sort(element1):
    return int(re.sub("\D", "", element1))



def gather_files_by_prefix(prefix:str, path):
    all_files = []
    files = os.listdir(path)
    for file in files:
        if file[0:len(prefix)] == prefix:
            all_files.append(file)
    all_files.sort(key=file_sort)
    return all_files


def collect_all_data_by_prefix(prefix:str):
    total_errors = 0
    path = "H:/extracted_folder/"
    files = gather_files_by_prefix(prefix, path)
    for file in files:
        full_path = path + file
        print(full_path)
        test_file = open(full_path, "rb")
        data = test_file.read()
        total_errors += process_data(data)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    #files = os.listdir()
    collect_all_data_by_prefix("ppg")

