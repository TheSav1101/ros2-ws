#include <hpe_test/data.hpp>
#include <opencv2/core/cvdef.h>
#include <sys/types.h>
#include <typeindex>
#include <vector>

const std::vector<std::string> MODEL_FILES = {
    "./src/hpe_test/models/singlepose-lightning-tflite-int8.tflite",
    "./src/hpe_test/models/singlepose-thunder-tflite-int8.tflite",
    "./src/hpe_test/models/singlepose-lightning-tflite-float16.tflite",
    "./src/hpe_test/models/singlepose-thunder-tflite-float16.tflite"};
const std::vector<int> MODEL_WIDTH = {192, 256, 192, 256};
const std::vector<int> MODEL_HEIGHT = {192, 256, 192, 256};
const std::vector<std::type_index> MODEL_INPUT = {
    typeid(u_int8_t), typeid(u_int8_t), typeid(cv::float16_t),
    typeid(cv::float16_t)};

const std::vector<std::string> DETECTION_MODEL_FILES = {
    "./src/hpe_test/detector_models/yolov4-tiny-416.tflite",
    "./src/hpe_test/detector_models/yolov4-tiny-416-int8.tflite"};
const std::vector<int> DETECTION_MODEL_WIDTH = {416, 416};
const std::vector<int> DETECTION_MODEL_HEIGHT = {416, 416};
const std::vector<std::type_index> DETECTOR_MODEL_INPUT = {typeid(float),
                                                           typeid(u_int8_t)};

const double MIN_SKELETON_CONFIDENCE = 0.5;
const double MIN_BOX_CONFIDENCE = 0.8;
