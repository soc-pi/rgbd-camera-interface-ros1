
# Variables
CONTAINER_NAME = ros1_dev_container
IMAGE_NAME = ros1-dev

.PHONY: build run stop clean deep-clean

# Build the Docker image
build:
	docker-compose build

# Run the container
run:
	docker-compose up -d
	docker-compose exec ros1-dev bash

# Stop the container
stop:
	docker-compose down

# Clean stopped containers and dangling images
clean:
	docker-compose down
	docker container prune -f
	docker image prune -f

# Deep clean - remove all related containers and images
deep-clean:
	docker-compose down
	docker container prune -f
	docker rmi $(IMAGE_NAME)
	docker image prune -f

# Show help
help:
	@echo "Available targets:"
	@echo "  build       - Build Docker image"
	@echo "  run         - Start container and attach to it"
	@echo "  stop        - Stop container"
	@echo "  clean       - Remove stopped containers and dangling images"
	@echo "  deep-clean  - Remove all related containers and images"
	@echo "  help        - Show this help message"
