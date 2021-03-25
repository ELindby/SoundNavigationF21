g++ main_test.cpp \
sources/motorcontrol.cpp \
sources/vision.cpp \
sources/odas.cpp \
sources/navigation.cpp \
sources/lidar.cpp \
sdk/include/rplidar.h \
sdk/src/rplidar_driver.cpp \
sdk/src/hal/thread.cpp \
sdk/src/arch/linux/timer.cpp \
sdk/src/arch/linux/net_serial.cpp \
sdk/src/arch/linux/net_socket.cpp \
-o main.out \
-Wall \
-pthread \
-L/usr/local/lib \
-ljson-c \
-I/usr/local/include/opencv4 \
-lm \
-lraspicam \
-lopencv_core \
-lopencv_features2d \
-lopencv_highgui \
-lopencv_imgcodecs \
-lopencv_imgproc \
-lmatrix_creator_hal \
-lgflags 


echo "To run, type:"
echo "./main.out"
echo "To run with ODASlive, type:"
echo "./runodasandmain.sh"
