# ==========================================
# AeroSim 3D - Multi-Platform Build System
# Engineered by Shivam Shukla
# ==========================================

# Compiler and Tools
CXX      = g++
TARGET   = AeroSim3D.exe

# Directory Structure
SRC_DIR  = src
INC_DIR  = include
BUILD_DIR = build
RES_DIR  = resources

# Compiler Flags
# -O2: Optimization level for performance
# -Wall: Enable all standard warnings
# -MMD -MP: Automatic header dependency tracking
# pkg-config: Fetches libcurl include paths dynamically
CXXFLAGS = -O2 -Wall -I$(INC_DIR) -D_DEFAULT_SOURCE -Wno-missing-braces -MMD -MP $(shell pkg-config --cflags libcurl)

# Linker Flags and Libraries
# Includes Raylib and the networking stack for the AI Director
LDLIBS   = -lraylib -lglfw3 -lopengl32 -lgdi32 -lwinmm -lcurl

# Source and Object File Mapping
SRCS     = $(wildcard $(SRC_DIR)/*.cpp)
OBJS     = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRCS))
DEPS     = $(OBJS:.o=.d)

# --- Build Rules ---

.PHONY: all clean setup

# Default target: Create build directory and compile executable
all: setup $(TARGET)

# Create the build directory if it does not exist
setup:
	@mkdir -p $(BUILD_DIR)

# Link the object files into the final executable
$(TARGET): $(OBJS)
	@echo "Linking executable: $@"
	$(CXX) -o "$@" $(OBJS) $(LDLIBS)

# Compile source files into object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo "Compiling: $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean target: Wipes the build folder and the executable for a fresh start
clean:
	@echo "Cleaning project binaries..."
	rm -rf $(BUILD_DIR) $(TARGET)

# Include generated dependency files
-include $(DEPS)