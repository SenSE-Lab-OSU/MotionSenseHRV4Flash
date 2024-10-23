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


def calculate_file_end(file):
    index = -1
    subtract_length = 0
    while file[index] == 0xff:
        subtract_length += 1
        index -= 1
    return subtract_length

struct_key = {"f":4,
              "h":2,
              "I": 4,
              "i": 4,
              "H":2,
              "Q": 8
              }

def process_data(data, categories: list[list], format: list, use_check=False) -> int:
    # we always assume that the last category is the packet counter
    #assert len(categories) == len(format)
    errors = 0
    
    skip_code = 1
    counter_value = 1
    current_index = 0
    data_position = 0
    num_of_categories = len(categories)
    end_trim_size = calculate_file_end(data)
    data_length = len(data) - end_trim_size
    while data_position + struct_key[format[current_index][1]] <= data_length:
        try:

            length = struct_key[format[current_index][1]]
            data_byte = data[data_position:data_position+length]
            data_position += length
            result = struct.unpack(format[current_index], data_byte)[0]

            if result == 4294967295:
                continue
            #print(result)
            categories[current_index].append(result)
            current_index += 1
            if current_index >= num_of_categories:
                current_index = 0
                if use_check:
                # we are on the last category, which means we are looking at the counter
                    if (result != counter_value):
                        print("error: got " + str(result) + " expected " + str(counter_value))
                        errors += 1
                    counter_value += 1


        except Exception as e:
            errors += 1
            print(e)

    print("errors: " + str(errors))
    return errors






def file_sort(element1:str):
    numeric_index = element1.find(it_prefix)
    numeric_time = element1[numeric_index+len(it_prefix):len(element1)]
    return int(re.sub("\D", "", numeric_time))



def gather_files_by_prefix(prefix:str, path):
    global it_prefix
    it_prefix = prefix
    all_files = []
    files = os.listdir(path)
    for file in files:
        if prefix in file:
            all_files.append(file)
    all_files.sort(key=file_sort)
    return all_files


def collect_all_data_by_prefix(path, prefix:str, labels:list[str], types:list[str]):
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
        total_errors += process_data(data, all_data, types)
    full_dict = {}
    for index in range(len(labels)):
        full_dict[labels[index]] = all_data[index]

    dataset = pd.DataFrame(full_dict)
    
    return dataset



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    #files = os.listdir()"C:/Users/mallory.115/Downloads/MSense4Left1/MSense4Left1/"
    path = "G:/"
    ppg_labels = ["g1", "g2", "ir1", "ir2", "counter"]
    ppg_formats = ["<i", "<i", "<i", "<i", "<i"]

    acc_labels = ["AccX", "AccY", "AccZ", "GC", "GyroX", "GyroY", "GyroZ", "Counter", "ENMO"]
    acc_formats = ["<h", "<h", "<h", "<h", "<f", "<f", "<f", "<f","<Q"]
    #data_set = collect_all_data_by_prefix(path, "ppg", ppg_labels)
    try:
        accel_data_set = collect_all_data_by_prefix(path, "ac", acc_labels, acc_formats)
        accel_data_set.to_csv("acceleration.csv")
        graph_generation.pd_graph_generation("ac", accel_data_set)
    except Exception as e:
        print(e)
    try:
        ppg_data_set = collect_all_data_by_prefix(path, "ppg", ppg_labels, ppg_formats)
        ppg_data_set.to_csv("ppg.csv")
        graph_generation.pd_graph_generation("ppg", ppg_data_set)
    except Exception as e:
        print(e)

    
    

    
    # then save it as a csv

