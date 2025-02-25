# Define the compiler
CXX = g++

# Define compilation flags
CXXFLAGS = -std=c++11 -g -Wall

# Define the output executable
TARGET = service_example

# List of source files
SRCS = service_client.cpp service_server.cpp main.cpp

# Generate object file names from source files
OBJS = $(SRCS:.cpp=.o)

# Default target: build the executable
all: $(TARGET)

# Rule to link object files into the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Rule to compile source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Include dependencies for header files
service_client.o: service_common.h
service_server.o: service_common.h
main.o: service_common.h

# Clean target: remove object files and executable
clean:
	rm -f $(OBJS) $(TARGET)

# Phony targets to avoid conflicts with filenames
.PHONY: all clean