#include <iostream>
#define REALSENSE_YAML_CPP
#include "realsense.h"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace rs2;
using namespace realsense;

int main(int argc, char **argv) {
    YAML::Node setting = YAML::LoadFile(argv[1]);

    Config config = setting["config"].as<Config>();
    RealSense rs;
    rs.connect(config);

    return 0;
}
