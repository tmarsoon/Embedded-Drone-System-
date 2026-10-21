CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS ?=
LDFLAGS ?=
BUILD_DIR ?= build
PYTHON ?= python3

CORE_NAMES := DynamicPathPlanning Physics Sensor PIDController PredictiveMaintenance RTOS Drone Statistics EnhancedCommunication
CORE_OBJECTS := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(CORE_NAMES)))

.PHONY: all test smoke research sanitize clean
all: $(BUILD_DIR)/drone_sim

$(BUILD_DIR):
	mkdir -p "$@"

$(BUILD_DIR)/%.o: Embedded\ Drone\ System/%.cpp Embedded\ Drone\ System/Simulation.h Embedded\ Drone\ System/PathPlanning.h | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -I"Embedded Drone System" -MMD -MP -c "$<" -o "$@"

$(BUILD_DIR)/test_main.o: tests/test_main.cpp Embedded\ Drone\ System/Simulation.h Embedded\ Drone\ System/PathPlanning.h | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -I"Embedded Drone System" -MMD -MP -c "$<" -o "$@"

$(BUILD_DIR)/drone_sim: $(CORE_OBJECTS) $(BUILD_DIR)/Main.o
	$(CXX) $(LDFLAGS) $^ -o "$@"

$(BUILD_DIR)/sim_tests: $(CORE_OBJECTS) $(BUILD_DIR)/test_main.o
	$(CXX) $(LDFLAGS) $^ -o "$@"

test: $(BUILD_DIR)/sim_tests $(BUILD_DIR)/drone_sim
	"$(BUILD_DIR)/sim_tests"
	$(PYTHON) tests/test_cli.py "$(BUILD_DIR)/drone_sim"

smoke: $(BUILD_DIR)/drone_sim
	"$(BUILD_DIR)/drone_sim" --output "$(BUILD_DIR)/smoke" --trace

research: $(BUILD_DIR)/drone_sim
	$(PYTHON) tools/research.py --binary "$(BUILD_DIR)/drone_sim" --output results/research --runs 30

sanitize:
	$(MAKE) BUILD_DIR=build/sanitize CXXFLAGS="-std=c++17 -O1 -g -Wall -Wextra -Wpedantic -fsanitize=address,undefined -fno-omit-frame-pointer" LDFLAGS="-fsanitize=address,undefined" test

clean:
	rm -rf -- "$(BUILD_DIR)"

-include $(wildcard $(BUILD_DIR)/*.d)
