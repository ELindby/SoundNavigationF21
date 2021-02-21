g++ main.cpp -o main.out \
-Wall \
-pthread \
-L/usr/local/lib \
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
