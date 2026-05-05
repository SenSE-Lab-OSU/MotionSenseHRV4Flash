#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <stdio.h>

namespace fs = std::filesystem;

bool compare_files(const std::string& file1, const std::string& file2) {
    std::ifstream f1(file1, std::ifstream::binary | std::ifstream::ate);
    std::ifstream f2(file2, std::ifstream::binary | std::ifstream::ate);

    if (f1.fail() || f2.fail()) {
        return false;
    }

    if (f1.tellg() != f2.tellg()) {
        return false;
    }

    f1.seekg(0, std::ifstream::beg);
    f2.seekg(0, std::ifstream::beg);

    return std::equal(std::istreambuf_iterator<char>(f1.rdbuf()),
        std::istreambuf_iterator<char>(),
        std::istreambuf_iterator<char>(f2.rdbuf()));
}

std::vector<std::string> get_files(const std::string& directory) {
    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path().string());
        }
    }
    return files;
}

int main(int argc, char* argv[]) {
    printf("Starting file comparison...\n");
    int errors = 0;
    std::string directory;
    if (argc > 1) {
        printf("Directory provided: %s\n", argv[1]);
        directory = argv[1];
    }
    else {
        std::cout << "No directory provided. Using default path. C:/samplecollection. \n";
        directory = "C:/Users/Devan/Downloads/test_files/test"; // Change this to your folder path
    }
    if (!fs::exists(directory)) {
        std::cout << "Directory does not exist.\n";
        return 1;
    }
    std::vector<std::string> files = get_files(directory);

    if (files.empty()) {
        std::cout << "No files found in directory.\n";
        return 1;
    }
    printf("number of files: %llu \n", files.size());
    for (size_t i = 1; i < files.size(); ++i) {
        if (!compare_files(files[0], files[i])) {
            std::cout << "Files are not identical.\n";
            std::cout << files[0];
            std::cout << files[1] << "\n";

            errors += 1;
        }
        else {
            std::cout << "Files are identical.\n";
        }

    }

    printf("non identical files: %i", errors);
    return 0;
}
