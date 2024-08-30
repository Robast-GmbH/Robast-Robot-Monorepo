#!/bin/bash
apt install iputils-ping -y
target_ip="192.168.0.200"

max_attempts=30
interval=5
attempts=0

while [[ $attempts -lt $max_attempts ]]; do
  ping -c 1 $target_ip > /dev/null 2>&1
  if [ $? -eq 0 ]; then
    echo "Ping to $target_ip was successful!"
    break
  else
    echo "Ping to $target_ip failed. Retrying in $interval seconds..."
    sleep $interval
  fi
  ((attempts++))
done

if [[ $attempts -eq $max_attempts ]]; then
  echo "Maximum number of attempts reached. Ping to $target_ip failed."
fi