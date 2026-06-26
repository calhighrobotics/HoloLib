#include "hololib/path.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

static constexpr float METER_TO_INCH = 39.3700787f;

/**
*@brief Parses path data from a string.
*@param input_source The string to parse.
*@param convertFromMeters Whether to convert from meters to inches.
*@return std::vector<PathPoint> The parsed path data.
*@note This function is used by the path following functions to parse path data
from a string. *The format of the input string should be as follows: x,y,theta
x,y,theta
x,y,theta
...
*/
std::vector<PathPoint> parsePathData(const std::string &input_source,
                                     bool convertFromMeters) {
  std::vector<PathPoint> path;
  const float scale = convertFromMeters ? METER_TO_INCH : 1.0f;

  bool is_file = (input_source.find('\n') == std::string::npos) &&
                 (input_source.find('/') != std::string::npos ||
                  input_source.find('.') != std::string::npos);

  auto processLine = [&](const std::string &raw) {
    std::string line = raw;
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' ||
                             line.back() == ' ' || line.back() == '\t'))
      line.pop_back();

    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos)
      return;
    line = line.substr(start);
    if (line.empty())
      return;

    std::stringstream ss(line);
    std::string cx, cy, ct;
    if (std::getline(ss, cx, ',') && std::getline(ss, cy, ',') &&
        std::getline(ss, ct, ',')) {
      try {
        PathPoint pt;
        pt.x = std::stof(cx) * scale;
        pt.y = std::stof(cy) * scale;
        pt.theta = std::stof(ct);
        path.push_back(pt);
      } catch (...) {
      }
    }
  };

  if (is_file) {
    std::ifstream file(input_source);
    if (!file.is_open()) {
      std::cout << "[PARSER ERROR] Cannot open: " << input_source << std::endl;
      return path;
    }
    std::string line;
    while (std::getline(file, line))
      processLine(line);
  } else {
    std::stringstream ss(input_source);
    std::string line;
    while (std::getline(ss, line))
      processLine(line);
  }

  std::cout << "[PARSER] Loaded " << path.size() << " points." << std::endl;
  return path;
}
