FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy project files
COPY pyproject.toml .
COPY ros2_agent/ ros2_agent/
COPY README.md .
COPY LICENSE .

# Install Python package
RUN pip install --no-cache-dir -e .

# Default environment
ENV ROS2_AGENT_MOCK_ROS2=true
ENV ROS2_AGENT_WEB_HOST=0.0.0.0
ENV ROS2_AGENT_WEB_PORT=8080

EXPOSE 8080

ENTRYPOINT ["ros2-agent"]
CMD ["web"]
