CXX = g++
TARGET = feature_matching
SRC = orb_slam.cpp
OPENCV_FLAGS = `pkg-config --cflags --libs opencv4`

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) -o $(TARGET) $(SRC) $(OPENCV_FLAGS)

clean:
	rm -f $(TARGET)