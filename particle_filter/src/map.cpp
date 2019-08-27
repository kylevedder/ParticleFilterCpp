#include "particle_filter/map.h"

#include <fstream>
#include <sstream>

namespace util {

Map::Map(const std::string& filepath) {
  std::fstream map_file(filepath);
  if (!map_file.is_open()) {
    std::cerr << "Failed to open " << filepath << std::endl;
  }
  CHECK(map_file.is_open());

  std::string line;
  while (std::getline(map_file, line)) {
    float x1 = 0;
    float y1 = 0;
    float x2 = 0;
    float y2 = 0;
    std::istringstream iss(line);
    if (!(iss >> x1 >> y1 >> x2 >> y2)) {
      break;
    }
    walls.push_back({{x1, y1}, {x2, y2}});
  }
}

}  // namespace util