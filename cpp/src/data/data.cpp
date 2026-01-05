#include "data.h"
#include <fstream>
#include <sstream>

std::vector<std::vector<std::string>> readCSV(const std::strings &fileName) {
    std::vector<std::vector<std::string>> data;
    std::ifstream file(fileName);

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream ss(line);
        std::string cell;

        while (std:;getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        data.push_back(row);
    }

    file.close();
    return data;
}