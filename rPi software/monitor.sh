#!/usr/bin/env bash

# ==========================================
# CONFIGURATION & THRESHOLDS
# ==========================================
# WiringPi pin 29
GPIO_PIN=29

# Threshold definitions
TEMP_WARN=65      # °C
TEMP_CRIT=70      # °C

CPU_WARN=85       # %
CPU_CRIT=95       # %

RAM_WARN=85       # %
RAM_CRIT=92       # %

CHECK_INTERVAL=2  # Sampling loop interval in seconds
# ==========================================

# Initialize WiringPi GPIO pin as output
gpio mode $GPIO_PIN out
gpio write $GPIO_PIN 0

# Cleanup on Ctrl+C or script termination
cleanup() {
    echo -e "\nStopping monitor service..."
    gpio write $GPIO_PIN 0
    exit 0
}
trap cleanup SIGINT SIGTERM

# Function to get CPU Temperature
get_cpu_temp() {
    local raw_temp
    if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
        raw_temp=$(cat /sys/class/thermal/thermal_zone0/temp)
        echo $((raw_temp / 1000))
    else
        echo 0
    fi
}

# Function to get CPU Load percentage over 1 second
get_cpu_load() {
    local cpu_now user nice system idle iowait irq softirq steal idle_now total_now
    read -r cpu_now user nice system idle iowait irq softirq steal _ < /proc/stat
    idle_now=$((idle + iowait))
    total_now=$((user + nice + system + idle + iowait + irq + softirq + steal))

    sleep 1

    local cpu_then user2 nice2 system2 idle2 iowait2 irq2 softirq2 steal2 idle_then total_then
    read -r cpu_then user2 nice2 system2 idle2 iowait2 irq2 softirq2 steal2 _ < /proc/stat
    idle_then=$((idle2 + iowait2))
    total_then=$((user2 + nice2 + system2 + idle2 + irq2 + softirq2 + steal2))

    local total_diff=$((total_then - total_now))
    local idle_diff=$((idle_then - idle_now))

    if [[ $total_diff -gt 0 ]]; then
        echo $(((total_diff - idle_diff) * 100 / total_diff))
    else
        echo 0
    fi
}

# Function to get RAM Usage percentage
get_ram_usage() {
    local total avail
    total=$(awk '/MemTotal:/ {print $2}' /proc/meminfo)
    avail=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
    
    if [[ $total -gt 0 ]]; then
        echo $(((total - avail) * 100 / total))
    else
        echo 0
    fi
}

# Function to get Root Disk Usage percentage
get_disk_usage() {
    df -h / | awk 'NR==2 {print $5}' | sed 's/%//'
}

# Beep pattern handler
trigger_beep() {
    local on_time=$1
    local off_time=$2
    local duration=$3
    
    # Calculate how many full cycles fit into the duration window
    # Bash uses integer math, so times are handled in milliseconds
    local on_ms=$(awk "BEGIN {print int($on_time * 1000)}")
    local off_ms=$(awk "BEGIN {print int($off_time * 1000)}")
    local cycle_ms=$((on_ms + off_ms))
    local total_ms=$((duration * 1000))
    local cycles=$((total_ms / cycle_ms))

    [[ $cycles -lt 1 ]] && cycles=1

    for ((i=0; i<cycles; i++)); do
        gpio write $GPIO_PIN 1
        sleep "$on_time"
        gpio write $GPIO_PIN 0
        sleep "$off_time"
    done
}

echo "Starting monitor on $GPIO_PIN..."

while true; do
    TEMP=$(get_cpu_temp)
    CPU=$(get_cpu_load)
    RAM=$(get_ram_usage)
    DISK=$(get_disk_usage)

    echo "[METRICS] CPU Temp: ${TEMP}°C | CPU Load: ${CPU}% | RAM Usage: ${RAM}% | Disk: ${DISK}%"

    # Evaluate severity
    if [[ $TEMP -ge $TEMP_CRIT || $CPU -ge $CPU_CRIT || $RAM -ge $RAM_CRIT ]]; then
        echo " -> ALERT: Critical threshold exceeded!"
        trigger_beep 0.50 0.50 $CHECK_INTERVAL
    elif [[ $TEMP -ge $TEMP_WARN || $CPU -ge $CPU_WARN || $RAM -ge $RAM_WARN ]]; then
        echo " -> ALERT: Warning threshold exceeded!"
        trigger_beep 0.25 0.75 $CHECK_INTERVAL
    else
        gpio write $GPIO_PIN 0
        sleep $CHECK_INTERVAL
    fi
done
