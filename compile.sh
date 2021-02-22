g++ main.cpp sources/motorcontrol.cpp sources/vision.cpp -o main.out \
-Wall \
-pthread \
-L/usr/local/lib \
-L/usr/include/json-c \
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
