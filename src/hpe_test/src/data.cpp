#include <hpe_test/data.hpp>

const std::vector<std::string>  MODEL_FILES             = {"./src/hpe_test/models/1.tflite", "./src/hpe_test/models/4.tflite"};
const std::vector<int>          MODEL_WIDTH             = {192, 256};
const std::vector<int>          MODEL_HEIGHT            = {192, 256};
const std::vector<std::string>  DETECTION_MODEL_FILES   = {"./src/hpe_test/detector_models/yolov4-tiny-416.tflite"};
const std::vector<int>          DETECTION_MODEL_WIDTH   = {416};
const std::vector<int>          DETECTION_MODEL_HEIGHT  = {416};