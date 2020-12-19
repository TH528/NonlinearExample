//  csv_writer.hpp
//  Created by TH on 2020/11/29.
//  Copyright © 2020 TH. All rights reserved.

#ifndef CSV_WRITER_H_
#define CSV_WRITER_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <valarray>

namespace csv {

template <class T>
class CSVWriter {
 public:
  explicit CSVWriter(std::string file_name)
    : file_name_(file_name) {}

  void Open() {
    writing_file_.open(file_name_, std::ios::out);
  }

  template <std::size_t N, std::size_t DATA_SIZE>
  void Write(const std::array<std::array<T, DATA_SIZE>, N>& data) {
    for (std::size_t i = 0; i < data[0].size(); i++) {
      for (std::size_t j = 0; j < data.size(); j++) {
        std::ostringstream oss;
        oss << std::scientific << data[j][i];
        std::string str_data = oss.str();

        writing_file_ << str_data;

        if (j != data.size() - 1) {
          writing_file_ << ",";
        }
      }

      writing_file_ << std::endl;
    }
  }

 private:
  std::string file_name_;
  std::ofstream writing_file_;
};

} // namespace csv

#endif