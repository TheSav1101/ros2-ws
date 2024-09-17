#ifndef HPE_TEST__DATA_
#define HPE_TEST__DATA_

std::vector<std::string>    MODEL_FILES             = {"./src/hpe_test/models/4.tflite", "./src/hpe_test/models/1.tflite"};
std::vector<int>            MODEL_WIDTH             = {192, 256};
std::vector<int>            MODEL_HEIGHT            = {192, 256};

std::vector<std::string>    DETECTION_MODEL_FILES   = {"./src/hpe_test/detector_models/yolov4-tiny-416.tflite"};
std::vector<int>            DETECTION_MODEL_WIDTH   = {416};
std::vector<int>            DETECTION_MODEL_HEIGHT  = {416};

#endif