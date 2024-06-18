import os
import struct
import re
import pandas as pd

import graph_generation


def process_data_test(data) -> int:

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
    #assert len(categories) == len(format)
    errors = 0
    
    skip_code = 1
    counter_value = 1
    current_index = 0
    num_of_categories = len(categories)
    for data_byte in range(0, len(data), 4):
        try:
            result = struct.unpack("<I", data[data_byte:data_byte+4])[0]
            if result == 4294967295:
                continue
            #print(result)
            categories[current_index].append(result)
            if current_index == num_of_categories -1:
                # we are on the last category, which means we are looking at the counter
                current_index = 0
                if (result != counter_value):
                    print("error: got " + str(result) + " expected " + str(counter_value))
                    errors += 1
                counter_value += 1
            else:
                current_index += 1
        except Exception as e:
            errors += 1
            print(e)

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


def collect_all_data_by_prefix(path, prefix:str, labels:list[str]):
    total_errors = 0
    
    all_data = []
    for element in range(len(labels)):
        all_data.append([])
    files = gather_files_by_prefix(prefix, path)
    for file in files:
        full_path = path + file
        print(full_path)
        test_file = open(full_path, "rb")
        data = test_file.read()
        total_errors += process_data(data, all_data, [])
        break
    full_dict = {}
    for index in range(len(labels)):
        full_dict[labels[index]] = all_data[index]

    dataset = pd.DataFrame(full_dict)
    #dataset.to_csv()
    return dataset



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    #files = os.listdir()
    path = "E:/"
    ppg_labels = ["g1", "g2", "ir1", "ir2", "counter"]
    data_set = collect_all_data_by_prefix(path, "ppg", ppg_labels)
    graph_generation.pd_graph_generation("ppg", data_set)
    # then save it as a csv

