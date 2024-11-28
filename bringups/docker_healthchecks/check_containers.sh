#!/bin/bash

# Log file for health check events
LOG_FILE="/home/robast/workspace/log/docker_healthcheck.log"

# Check the health status of running Docker containers
check_containers() {
    containers=$(docker ps --filter "health=unhealthy" --format "{{.Names}}")

    for container in $containers; do
        echo "Restarting unhealthy container: $container"
        docker restart "$container"
        echo "$(date '+%Y-%m-%d %H:%M:%S') - Restarted container: $container" >> "$LOG_FILE"
    done
}

# Execute the health check
while true; do
    check_containers
    sleep 1
done