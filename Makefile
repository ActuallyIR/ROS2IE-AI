.PHONY: install run dev test lint

install:
	pip install -e ".[dev]"

## Run the web UI (frontend + backend) with mock ROS 2
run:
	python -m ros2_agent web --mock

## Run with auto-reload for development
dev:
	python -m ros2_agent web --mock --reload

## Run on a custom port
run-port:
	python -m ros2_agent web --mock --port $(PORT)

test:
	pytest

lint:
	ruff check ros2_agent tests
	mypy ros2_agent
