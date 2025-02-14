# Build directory
BUILD_DIR = build
DOCKER_IMG ?= noetic-ros1-dev
DOCKER_TAG ?= latest

# Host-specific settings
USERID := $(shell id -u)
GROUPID := $(shell id -g)

# Docker workspace path
DOCKER_WS = /ws/src/rgbd-camera-interface

# Default target now uses Docker
all: docker-build-check docker-build-cmd

# Local build target (without Docker)
local-build:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake .. && make -j$$(nproc)

# Ensure Docker image exists
docker-build-check:
	@if ! docker images $(DOCKER_IMG):$(DOCKER_TAG) | grep -q $(DOCKER_IMG); then \
		echo "Docker image not found. Building first..."; \
		$(MAKE) docker-build || exit 1; \
	fi

# Run build command in Docker
docker-build-cmd: docker-build-check clean 
	@echo "Building project in Docker container..."
	@docker run --rm \
		--user $(USERID):$(GROUPID) \
		--network=host \
		-v $(PWD):$(DOCKER_WS) \
		$(DOCKER_IMG):$(DOCKER_TAG) \
		bash -c "source /opt/ros/noetic/setup.bash && \
			cd $(DOCKER_WS) && \
			mkdir -p $(BUILD_DIR) && \
			cd $(BUILD_DIR) && \
			cmake .. && \
			make -j$$(nproc)" || \
	(echo "Docker build failed!" && exit 1)

# Clean build artifacts
clean:
	@rm -rf $(BUILD_DIR)

# Clean and rebuild
rebuild: clean all

# Override other targets to use Docker
install: docker-build-check
	docker run --rm \
		--user $(USERID):$(GROUPID) \
		-v $(PWD):$(DOCKER_WS) \
		$(DOCKER_IMG):$(DOCKER_TAG) \
		bash -c "source /opt/ros/noetic/setup.bash && \
			cd $(DOCKER_WS)/$(BUILD_DIR) && \
			make install"

test: docker-build-check
	docker run --rm \
		--user $(USERID):$(GROUPID) \
		-v $(PWD):$(DOCKER_WS) \
		$(DOCKER_IMG):$(DOCKER_TAG) \
		bash -c "source /opt/ros/noetic/setup.bash && \
			cd $(DOCKER_WS)/$(BUILD_DIR) && \
			ctest --output-on-failure"

# Enhanced Docker targets
docker-build:
	@echo "Building Docker image $(DOCKER_IMG):$(DOCKER_TAG)..."
	@docker build \
		--build-arg USER_ID=$(USERID) \
		--build-arg GROUP_ID=$(GROUPID) \
		-t $(DOCKER_IMG):$(DOCKER_TAG) . || \
	(echo "Failed to build Docker image!" && exit 1)

docker-run:
	docker run -it --rm \
		--user $(USERID):$(GROUPID) \
		--network=host \
		--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--device=/dev/video0:/dev/video0 \
		-v $(PWD):$(DOCKER_WS) \
		$(DOCKER_IMG):$(DOCKER_TAG)

docker-shell:
	docker run -it --rm \
		--user $(USERID):$(GROUPID) \
		--network=host \
		--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--device=/dev/video0:/dev/video0 \
		-v $(PWD):$(DOCKER_WS) \