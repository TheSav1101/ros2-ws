FROM ros:jazzy-perception-noble

RUN apt-get update && apt-get upgrade -y && apt-get install wget

RUN wget https://github.com/bazelbuild/bazel/releases/download/6.1.0/bazel-6.1.0-linux-arm64 && cp bazel-6.1.0-linux-arm64 /usr/bin/

##Install TFLITE
RUN git clone https://github.com/tensorflow/tensorflow.git tensorflow_src

RUN cd tenorflow_src && bazel build --config=elinux_aarch64 -c opt //tensorflow/lite:libtensorflowlite.so
RUN cd tenorflow_src && bazel build --config=elinux_aarch64 -c opt //tensorflow/lite:libtensorflowlite_c.so

RUN cd && cp /tensorflow_src/btensorflowlite.so /usr/lib/libtensorflowlite.so 
RUN cd && cp /tensorflow_src/btensorflowlite_c.so /usr/lib/libtensorflowlite_c.so 

RUN mkdir /usr/lib/cmake/tensorflowlite && touch /usr/lib/cmake/tensorflowlite/tensorflowlite-config.cmake \
       && echo ' #Configure file for tensorflow_lite \
      #set(TENSORFLOW_LITE_LIBRARIES "/usr/lib/libtensorflowlite.so) \ 
      set(TENSORFLOW_LITE_C_LIBRARIES "/usr/lib/libtensorflowlite_c.so") \
      set(tensorflow_lite_INCLUDE_DIRS "/usr/include/tensorflow") \
      set(tensorflow_lite_LIBRARIES "/usr/lib/libtensorflowlite.so") \
      set(tensorflow_lite_INCLUDE_DIRS  CACHE STRING "tensorflow_lite include directories" FORCE) \
      set(tensorflow_lite_LIBRARIES  CACHE STRING "tensorflow_lite libraries" FORCE)' > /usr/lib/cmake/tensorflowlite/tensorflowlite-config.cmake


##Would be nice to test stuff here
