#!/usr/bin/env python3

import psutil
import time
import csv
import subprocess
import threading
from datetime import datetime
import argparse

# Optional: NVIDIA GPU monitoring via nvidia-smi
USE_GPU = True
GPU_QUERY_CMD = [
    'nvidia-smi',
    '--query-gpu=utilization.gpu,memory.used',
    '--format=csv,noheader,nounits'
]

# Network interface to monitor
NET_IFACE = 'eno1'  # Change this to 'wlan0' or the interface you use

# Output CSV file
FILENAME = f"system_monitoring_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Time interval between samples (seconds)
INTERVAL = 1.0

# Header for the CSV
CSV_HEADERS = [
    'timestamp', 'cpu_percent', 'mem_percent',
    'net_sent_MBps', 'net_recv_MBps', 'latency_ms'
]
if USE_GPU:
    CSV_HEADERS.extend(['gpu_util_percent', 'gpu_mem_used_MB'])


def get_latency(host='8.8.8.8'):
    try:
        out = subprocess.check_output(['ping', '-c', '1', '-W', '1', host], stderr=subprocess.DEVNULL)
        for line in out.decode().split('\n'):
            if 'time=' in line:
                return float(line.split('time=')[1].split()[0])
    except Exception:
        return None


def get_gpu_stats():
    try:
        output = subprocess.check_output(GPU_QUERY_CMD).decode().strip()
        gpu_util, mem_used = map(str.strip, output.split(','))
        return float(gpu_util), float(mem_used)
    except Exception:
        return None, None


def monitor_system(output_file):
    # Initialize previous net counters
    prev = psutil.net_io_counters(pernic=True)[NET_IFACE]
    prev_time = time.time()

    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(CSV_HEADERS)

        while True:
            now = time.time()
            timestamp = datetime.now().isoformat()
            cpu = psutil.cpu_percent()
            mem = psutil.virtual_memory().percent

            curr = psutil.net_io_counters(pernic=True)[NET_IFACE]
            elapsed = now - prev_time
            sent_MBps = (curr.bytes_sent - prev.bytes_sent) / (1024 ** 2) / elapsed
            recv_MBps = (curr.bytes_recv - prev.bytes_recv) / (1024 ** 2) / elapsed
            prev, prev_time = curr, now

            latency = get_latency()
            row = [
                timestamp,
                cpu,
                mem,
                round(sent_MBps, 3),
                round(recv_MBps, 3),
                round(latency, 3) if latency else 'N/A'
            ]

            if USE_GPU:
                gpu_util, gpu_mem = get_gpu_stats()
                row.extend([
                    round(gpu_util, 1) if gpu_util is not None else 'N/A',
                    round(gpu_mem, 1) if gpu_mem is not None else 'N/A'
                ])

            writer.writerow(row)
            csvfile.flush()
            time.sleep(INTERVAL)

def parse_args():
    parser = argparse.ArgumentParser(description="System monitoring tool")
    parser.add_argument("--output", type=str, required=True, help="Path to output CSV file")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    monitoring_active = True
    try:
        monitor_system(args.output)
    except KeyboardInterrupt:
        monitoring_active = False
        print("Monitoring stopped by user.")
