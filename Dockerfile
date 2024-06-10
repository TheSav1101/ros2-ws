FROM --platform=linux/arm64 ros:jazzy-perception-noble
#FROM ros:jazzy-perception-noble


SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get upgrade -y
RUN apt-get install ros-jazzy-cv-bridge -y

##Install TFLITE
#RUN git clone https://github.com/tensorflow/tensorflow.git tensorflow_src

COPY ./libtensorflowlite.so /usr/lib/libtensorflowlite.so
COPY ./libtensorflowlite_c.so /usr/lib/libtensorflowlite_c.so

RUN mkdir /usr/lib/cmake/tensorflowlite && touch /usr/lib/cmake/tensorflowlite/tensorflowlite-config.cmake \
       && echo ' #Configure file for tensorflow_lite \
      #set(TENSORFLOW_LITE_LIBRARIES "/usr/lib/libtensorflowlite.so) \ 
      set(TENSORFLOW_LITE_C_LIBRARIES "/usr/lib/libtensorflowlite_c.so") \
      set(tensorflow_lite_INCLUDE_DIRS "/usr/include/tensorflow") \
      set(tensorflow_lite_LIBRARIES "/usr/lib/libtensorflowlite.so") \
      set(tensorflow_lite_INCLUDE_DIRS  CACHE STRING "tensorflow_lite include directories" FORCE) \
      set(tensorflow_lite_LIBRARIES  CACHE STRING "tensorflow_lite libraries" FORCE)' > /usr/lib/cmake/tensorflowlite/tensorflowlite-config.cmake

COPY ./src /ros2-ws/src

RUN echo 'source /opt/ros/jammy/setup.sh' > /home/ubuntu/.bashrc
RUN echo 'source /ros2-ws/install/local_setup.sh' > /home/ubuntu/.bashrc

WORKDIR "/ros2-ws"

RUN chmod +x /opt/ros/jazzy/setup.sh
RUN /opt/ros/jazzy/setup.sh && colcon build #--package-select hpe_msgs
RUN chmod +x /ros2-ws/install/local_setup.sh 
RUN colcon build


ENTRYPOINT ["bash"]

##Would be nice to test stuff here
