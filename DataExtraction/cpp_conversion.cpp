#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <filesystem>
#include <regex>
#include <Windows.h>

namespace fs = std::filesystem;

int calculate_file_end(const std::vector<uint8_t>& file) {
    int index = -1;
    int subtract_length = 0;
    while (file[file.size() + index] == 0xff) {
        subtract_length += 1;
        index -= 1;
    }
    return subtract_length;
}

std::unordered_map<std::string, int> struct_key = {
    {"f", 4},
    {"h", 2},
    {"I", 4},
    {"i", 4},
    {"H", 2},
    {"Q", 8}
};



void resultant_end_trim_workaround(std::vector<std::vector<uint64_t>>& categories) {
    std::vector<int> length_array;
    for (const auto& element : categories) {
        length_array.push_back(element.size());
    }

    int max_diff = *std::max_element(length_array.begin(), length_array.end()) - *std::min_element(length_array.begin(), length_array.end());
    if (max_diff == 0) {
        return;
    }
    if (max_diff == 1) {
        std::cout << "warning: end length of array differs by 1. Implementing fix." << std::endl;
        std::cout << "mismatch array: ";
        for (const auto& len : length_array) {
            std::cout << len << " ";
        }
        std::cout << std::endl;
        int max_value = *std::max_element(length_array.begin(), length_array.end());
        for (auto& element : categories) {
            if (max_value == element.size()) {
                element.pop_back();
            }
        }
    }
    else if (max_diff > 1) {
        std::cerr << "error: end lengths of array differs by too much. Data is potentially corrupt." << std::endl;
        std::cout << "mismatch array: ";
        for (const auto& len : length_array) {
            std::cout << len << " ";
        }
        std::cout << std::endl;
    }
}


int process_data(const std::vector<uint8_t>& data, std::vector<std::vector<uint64_t>>& categories, const std::vector<std::pair<std::string, std::string>>& format, bool use_check = false) {
    int errors = 0;
    int skip_code = 1;
    uint64_t counter_value = 1;
    int current_index = 0;
    int data_position = 0;
    int num_of_categories = categories.size();
    int end_trim_size = calculate_file_end(data);
    int data_length = data.size() - end_trim_size;

    while (data_position + struct_key[format[current_index].second] <= data_length) {
        try {
            int length = struct_key[format[current_index].second];
            std::vector<uint8_t> data_byte(data.begin() + data_position, data.begin() + data_position + length);
            data_position += length;
            uint64_t result = 0;

            if (format[current_index].second == "Q") {
                result = *reinterpret_cast<const uint64_t*>(data_byte.data());
            }
            else if (format[current_index].second == "I" || format[current_index].second == "i") {
                result = *reinterpret_cast<const uint32_t*>(data_byte.data());
            }
            else if (format[current_index].second == "H" || format[current_index].second == "h") {
                result = *reinterpret_cast<const uint16_t*>(data_byte.data());
            }

            if (result == 4294967295) {
                continue;
            }
            categories[current_index].push_back(result);
            current_index += 1;
            if (current_index >= num_of_categories) {
                current_index = 0;
                if (use_check) {
                    if (result != counter_value) {
                        std::cerr << "error: got " << result << " expected " << counter_value << std::endl;
                        errors += 1;
                    }
                    counter_value += 1;
                }
            }
        }
        catch (const std::exception& e) {
            errors += 1;
            std::cerr << e.what() << std::endl;
        }
    }

    if (categories[0].size() > categories[1].size()) {
        std::cout << "0xff-trim issue found, fixing..." << std::endl;
        categories[0].pop_back();
    }

    resultant_end_trim_workaround(categories);

    std::cout << "errors: " << errors << std::endl;
    return errors;
}


int file_sort(const std::string& element1) {
    std::string it_prefix;
    size_t numeric_index = element1.find(it_prefix);
    std::string numeric_time = element1.substr(numeric_index + it_prefix.length());
    return std::stoi(std::regex_replace(numeric_time, std::regex("\\D"), ""));
}

std::vector<std::string> gather_files_by_prefix(const std::string& prefix, const std::string& path) {
    std::string it_prefix = prefix;
    std::vector<std::string> all_files;
    for (const auto& entry : fs::directory_iterator(path)) {
        if (entry.path().filename().string().find(prefix) != std::string::npos) {
            all_files.push_back(entry.path().filename().string());
        }
    }
    std::sort(all_files.begin(), all_files.end(), [](const std::string& a, const std::string& b) {
        return file_sort(a) < file_sort(b);
        });
    return all_files;
}

std::vector<std::string> obtain_prefix_ids(const std::string& path) {
    std::vector<std::string> all_files;
    for (const auto& entry : fs::directory_iterator(path)) {
        std::string filename = entry.path().filename().string();
        if (std::isdigit(filename[0])) {
            std::smatch match;
            if (std::regex_search(filename, match, std::regex(R"(\d+)"))) {
                std::string id = match.str();
                if (std::find(all_files.begin(), all_files.end(), id) == all_files.end()) {
                    all_files.push_back(id);
                }
            }
        }
    }
    return all_files;
}

