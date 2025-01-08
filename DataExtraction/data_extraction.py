import os
import sys
import struct
import re
import datetime

try:
    import numpy
    import pandas as pd
    import graph_generation
except ImportError:
    print(
        "unable to import packages, please install numpy or pandas if you would like to use the graph functionality!")


def process_data_test(data) -> int:
    errors = 0
    check_array = [1, 2, 3, 4, 5, 6]
    for data_byte in range(0, len(data), 2):
        result = struct.unpack("<h", data[data_byte:data_byte + 2])[0]
        # print(result)
        inx = data_byte // 2 % 6
        if (result != check_array[inx]):
            # print("error: got " + str(result[0]) + " expected " + str(check_array[data_byte//2 % 6]))
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


struct_key = {"f": 4,
              "h": 2,
              "I": 4,
              "i": 4,
              "H": 2,
              "Q": 8
              }


def process_data(data, categories: list[list], format: list, use_check=False) -> int:
    # we always assume that the last category is the packet counter
    # assert len(categories) == len(format)
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
            data_byte = data[data_position:data_position + length]
            data_position += length
            result = struct.unpack(format[current_index], data_byte)[0]

            if result == 4294967295:
                continue
            # print(result)
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

    if len(categories[0]) > len(categories[1]):
        print("0xff-trim issue found, fixing...")
        categories[0].pop()

    resultant_end_trim_workaround(categories)

    print("errors: " + str(errors))
    return errors


def resultant_end_trim_workaround(categories):
    length_array = []
    for element in categories:
        length_array.append(len(element))

    max_diff = numpy.max(length_array) - numpy.min(length_array)
    if max_diff == 0:
        return
    if max_diff == 1:
        print("warning: end length of array differs by 1. Implementing fix.")
        print("mismatch array: " + str(length_array))
        max_value = max(length_array)
        for element in categories:
            if max_value == len(element):
                element.pop()
    elif max_diff > 1:
        print("error: end lengths of array differs by too much. Data is potentially corrupt.")
        print("mismatch array: " + str(length_array))


def file_sort(element1: str):
    numeric_index = element1.find(it_prefix)
    numeric_time = element1[numeric_index + len(it_prefix):len(element1)]
    return int(re.sub("\D", "", numeric_time))


def gather_files_by_prefix(prefix: str, path):
    global it_prefix
    it_prefix = prefix
    all_files = []
    files = os.listdir(path)
    for file in files:
        if prefix in file:
            all_files.append(file)
    all_files.sort(key=file_sort)
    return all_files


def obtain_prefix_ids(path):
    all_files = []
    files = os.listdir(path)
    for file in files:
        if file[0].isdigit():
            id = re.search(r'\d+', file)
            if id is not None:
                id = id.group()
                if id not in all_files:
                    all_files.append(id)

    return all_files


def counter_validity_check(df: pd.DataFrame):
    try:
        counter_columns = df.iloc[:, -1:]
        counter_arr = numpy.array(counter_columns).flatten()
        diff_arr = numpy.diff(counter_arr)
        check_array = (diff_arr == 8) | (diff_arr == -65528)
        print("pass counter check: " + str(numpy.all(check_array)))
        print("and number of non matching samples: " + str(numpy.count_nonzero(check_array == 0)))
    except Exception as e:
        print(e)


def collect_all_data_by_prefix(path, prefix: str, labels: list[str], types: list[str]):
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
        if len(data) != 0:
            total_errors += process_data(data, all_data, types)
        else:
            print("Warning: found empty file!")
    full_dict = {}
    for index in range(len(labels)):
        full_dict[labels[index]] = all_data[index]

    dataset = pd.DataFrame(full_dict)

    return dataset


def generate_csv_for_pattern(file_prefix, type_prefix: str, search_key: str, labels, formats):
    try:
        file_name = file_prefix + str(search_key) + "_at_" + str(int(datetime.datetime.now().timestamp()))

        file_name += type_prefix
        data_set = collect_all_data_by_prefix(path, search_key, labels, formats)
        data_set.to_csv(file_name)
        counter_validity_check(data_set)
        graph_generation.pd_graph_generation(search_key, data_set)

    except Exception as e:
        print(e)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    path = "F:/"  # "C:/Users/mallory.115/Downloads/Left1_drive/Left1_drive/" #"F:/" #"D:/8088-5/8088-5/11122024/"
    if len(sys.argv) >= 2:
        file_prefix = sys.argv[1]
        if len(sys.argv) >= 3:
            path = sys.argv[2]
    else:
        file_prefix = ""
    # files = os.listdir()"C:/Users/mallory.115/Downloads/MSense4Left1/MSense4Left1/"

    ppg_labels = ["g1", "g2", "ir1", "ir2", "counter"]
    ppg_formats = ["<i", "<i", "<i", "<i", "<i"]

    acc_labels = ["AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "ENMO", "Counter", ]
    acc_formats = ["<h", "<h", "<h", "<f", "<f", "<f", "<f", "<i"]

    ids = obtain_prefix_ids(path)

    ids.append("")

    for id in ids:
        search_prefix = id + "ac"
        file_name = search_prefix + ".csv"
        generate_csv_for_pattern(file_prefix, file_name, search_prefix, acc_labels, acc_formats)
        search_prefix = id + "ppg"
        file_name = search_prefix + ".csv"
        generate_csv_for_pattern(file_prefix, file_name, search_prefix, ppg_labels, ppg_formats)

    # then save it as a csv

