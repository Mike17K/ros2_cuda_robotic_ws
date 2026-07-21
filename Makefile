# --- Config ---
SHELL      := /bin/bash
ROS_DISTRO := jazzy
WS_ROOT    := $(shell pwd)
RUN        := cd $(WS_ROOT) && source /opt/ros/$(ROS_DISTRO)/setup.bash && export ISAAC_ROS_WS=$(WS_ROOT) && 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/hpcx/ucx/lib:/opt/hpcx/ucc/lib

# Colors
G=\033[0;32m
Y=\033[0;33m
R=\033[0;31m
C=\033[0;36m
RESET=\033[0m

.PHONY: all build debug builds pkg clean deps create-cpp create-py dev

CMAKE_DEFAULT_FLAGS = -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 

# 1. Build
all: build

run:
	@echo -e "$(C)Running Isaac ROS...$(RESET)"
	$(RUN)

rosdeps:
	@echo -e "$(C)Installing rosdeps...$(RESET)"
	rosdep update && rosdep install -i -r --from-paths src/ --rosdistro jazzy -y

build:
	$(RUN) colcon build \
	--symlink-install \
	--parallel-workers 2 \
	--base-paths src \
	--cmake-args $(CMAKE_DEFAULT_FLAGS) -DCMAKE_BUILD_TYPE=Release

# 2. Build Single Package (make builds n=όνομα)
builds:
	@if [ -z "$(n)" ]; then echo -e "$(R)Error: Provide name (n=name)$(RESET)"; exit 1; fi
	$(RUN) colcon build --packages-select $(n) --symlink-install --base-paths src --cmake-args $(CMAKE_DEFAULT_FLAGS) -DCMAKE_BUILD_TYPE=Release

# 3. Δημιουργία Πακέτων
create-cpp:
	@if [ -z "$(n)" ]; then echo -e "$(R)Error: Provide name (n=name)$(RESET)"; exit 1; fi
	cd src && $(RUN) ros2 pkg create --build-type ament_cmake --destination-directory src $(n)

create-py:
	@if [ -z "$(n)" ]; then echo -e "$(R)Error: Provide name (n=name)$(RESET)"; exit 1; fi
	cd src && $(RUN) ros2 pkg create --build-type ament_python --destination-directory src $(n)

# 4. Debug & Deps
debug:
	$(RUN) colcon build --symlink-install --base-paths src --cmake-args $(CMAKE_DEFAULT_FLAGS) -DCMAKE_BUILD_TYPE=Debug

debugs:
	$(RUN) colcon build --packages-select $(n) --symlink-install --base-paths src --cmake-args $(CMAKE_DEFAULT_FLAGS) -DCMAKE_BUILD_TYPE=Debug

deps:
	$(RUN) rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y

# 5. Maintenance
clean:
	@echo -e "$(Y)Cleaning workspace...$(RESET)"
	rm -rf build/ install/ log/
