#!/usr/bin/env python3
import asyncio
import struct
import datetime
import csv
import sys
import logging
from pathlib import Path

from bleak import BleakScanner, BleakClient, BleakError

# ——— UUIDs must match your Arduino definitions ———
TARGET_NAME      = "AAX"
SCAN_TIMEOUT     = 10  # seconds

SENSOR_SERVICE   = "290bd33f-8e7e-485b-b18a-6f8d28482d75"
AD_SERVICE       = "390bd33f-8e7e-485b-b18a-6f8d28482d75"

NOTIFY_CHAR      = "290bd33f-8e7e-485b-b18a-6f8d28482d76"
S_WRITE_CHAR     = "290bd33f-8e7e-485b-b18a-6f8d28482d77"
AD_WRITE_CHAR    = "390bd33f-8e7e-485b-b18a-6f8d28482d78"

# BLE payload limits
BLE_MTU          = 247
BLE_OVERHEAD     = 7
BLE_PACKET_SIZE  = BLE_MTU - BLE_OVERHEAD

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


class DataCollector:
    def __init__(self, choice:int, odr:int, duration:int):
        self.choice = choice
        self.odr = odr
        self.duration = duration

        # Determine unpack format
        if choice == 1:
            self.unpack_fmt = "<hhh"     # KX134: 3×int16
            self.fields     = ["ax","ay","az"]
        elif choice == 2:
            self.unpack_fmt = "<hhhhhh"  # IMU: 6×int16
            self.fields     = ["ax","ay","az","gx","gy","gz"]
        else:
            self.unpack_fmt = "<h"       # PDM: 1×int16
            self.fields     = ["audio"]

        self.bytes_per_sample = struct.calcsize(self.unpack_fmt)
        self.total_samples    = odr * duration
        self.total_bytes      = self.bytes_per_sample * self.total_samples

        # Buffers and counters
        self.buffer = bytearray()
        self.count  = 0

        # Prepare CSV file
        desktop = Path.home() / "Desktop"
        today_str = datetime.datetime.now().strftime("%Y%m%d")
        folder   = desktop / f"AAX_SensorData_{today_str}"
        folder.mkdir(exist_ok=True)
        fname    = folder / f"{self.fields[0]}_{odr}Hz_{duration}s_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
        self.csv = open(fname, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv)
        self.writer.writerow(self.fields + ["date","time"])
        logging.info(f"Logging to CSV: {fname}")

    def process_buffer(self):
        while len(self.buffer) >= self.bytes_per_sample and self.count < self.total_samples:
            chunk = self.buffer[:self.bytes_per_sample]
            del self.buffer[:self.bytes_per_sample]
            try:
                vals = struct.unpack(self.unpack_fmt, chunk)
            except struct.error as e:
                logging.warning(f"Unpack error: {e}, skipping sample")
                continue

            self.count += 1
            now = datetime.datetime.now()
            row = list(vals) + [now.date().isoformat(), now.time().isoformat(timespec="milliseconds")]
            self.writer.writerow(row)

            # Progress every 10%
            if self.count % max(1, self.total_samples // 10) == 0:
                pct = 100 * self.count / self.total_samples
                logging.info(f"Progress: {pct:.0f}% ({self.count}/{self.total_samples})")

    def notification_handler(self, sender: int, data: bytearray):
        logging.debug(f"Notify: {len(data)} bytes")
        self.buffer.extend(data)
        self.process_buffer()
        return self.count >= self.total_samples

    def close(self):
        self.csv.close()
        logging.info("CSV file closed.")


async def configure_sensor(client: BleakClient):
    """Let user pick sensor, ODR, duration, then start and collect data."""
    print("\nSelect Sensor to Sample:")
    print("  [1] KX-134 (3 axes)")
    print("  [2] IMU    (6 axes)")
    print("  [3] PDM    (audio)")
    choice = int(input("Choice (1–3): "))
    assert choice in (1,2,3)

    # Preset ODR & max durations
    if choice == 1:
        odr_map = {1:(8000,4), 2:(5000,6), 3:(12800,2)}
        odr_opt = int(input("KX-134 ODR options: [1]8kHz/4s [2]5kHz/6s [3]12.8kHz/2s → "))
        odr, max_dur = odr_map[odr_opt]
    elif choice == 2:
        odr, max_dur = 2000, 6
    else:
        odr, max_dur = 16000, 6

    duration = int(input(f"Duration (1–{max_dur}s): "))
    assert 1 <= duration <= max_dur

    # Pack and send config: <BHH>
    pkt = struct.pack("<BHH", choice, odr, duration)
    await client.write_gatt_char(S_WRITE_CHAR, pkt, response=True)
    logging.info(f"Sent sensor config: choice={choice}, odr={odr}, duration={duration}")

    # Set up collector
    collector = DataCollector(choice, odr, duration)

    # Start sampling
    input("Press Enter to START sampling…")
    await client.write_gatt_char(S_WRITE_CHAR, b"\x01", response=True)
    logging.info("Sampling start command sent")

    # Subscribe to notifications
    await client.start_notify(NOTIFY_CHAR, collector.notification_handler)
    logging.info("Notifications enabled, collecting data…")

    # Wait until done
    while collector.count < collector.total_samples:
        await asyncio.sleep(0.5)

    await client.stop_notify(NOTIFY_CHAR)
    collector.close()
    logging.info("Sampling session complete.")


async def configure_advertising(client: BleakClient):
    """Let user optionally set advertising/sleep times."""
    print("\nConfigure Advertising & Sleep (or leave default):")
    if input("Configure? (y/N): ").lower() != "y":
        return

    ad_time    = int(input("Advertising time (ms, 0–65535): "))
    sleep_time = int(input("Sleep time     (s,  0–65535): "))
    pkt = struct.pack("<HH", ad_time, sleep_time)
    await client.write_gatt_char(AD_WRITE_CHAR, pkt, response=True)
    logging.info(f"Sent AD config: adv={ad_time}ms, sleep={sleep_time}s")


async def main():
    logging.info(f"Scanning for '{TARGET_NAME}' for {SCAN_TIMEOUT}s…")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    target = next((d for d in devices if d.name == TARGET_NAME), None)
    if not target:
        logging.error(f"No device named '{TARGET_NAME}' found.")
        sys.exit(1)

    logging.info(f"Found {TARGET_NAME} @ {target.address}, connecting…")
    try:
        async with BleakClient(target.address) as client:
            if not client.is_connected:
                raise BleakError("Connection failed")

            logging.info("Connected!")
            # await client.request_mtu(BLE_MTU)

            print("\nSelect Service:")
            print("  [1] Sensor data sampling")
            print("  [2] Advertising / sleep config")
            svc = int(input("Choice (1–2): "))
            assert svc in (1,2)

            if svc == 1:
                await configure_sensor(client)
            else:
                await configure_advertising(client)

    except Exception as e:
        logging.error(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("Interrupted by user, exiting.")
        sys.exit(0)
