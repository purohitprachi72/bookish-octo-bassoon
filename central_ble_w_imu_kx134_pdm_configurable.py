import asyncio
import struct
import datetime
import csv
import os
from bleak import BleakScanner, BleakClient

TARGET_NAME = "AAX"
SCAN_TIMEOUT = 10  # seconds

SENSOR_SERVICE = "290bd33f-8e7e-485b-b18a-6f8d28482d75"
AD_SERVICE = "390bd33f-8e7e-485b-b18a-6f8d28482d75"

NOTIFY_CHAR = "290bd33f-8e7e-485b-b18a-6f8d28482d76"
S_WRITE_CHAR = "290bd33f-8e7e-485b-b18a-6f8d28482d77"
AD_WRITE_CHAR = "390bd33f-8e7e-485b-b18a-6f8d28482d78"

desktop = os.path.join(os.path.join(os.environ['USERPROFILE']), 'Desktop')
if not os.path.exists(desktop):
    print("Desktop path does not exist.")
today = datetime.datetime.now().strftime("%d-%m-%Y")
folder_name = f"nRF52S_SensorData_{today}"
BASE_PATH = os.path.join(desktop, folder_name)
if not os.path.exists(BASE_PATH):
    os.makedirs(BASE_PATH)
    print(f"Created directory: {BASE_PATH}")
else:
    print(f"Directory already exists: {BASE_PATH}")

class DataCollector:
    def __init__(self, sensor_choice, odr, duration):
        self.sensor_choice = sensor_choice  # 1: KX134, 2: IMU, 3: PDM
        self.odr = odr
        self.duration = duration
        
        # Calculate expected data sizes
        if sensor_choice == 1:  # KX134 - 3 axes
            self.bytes_per_sample = 6  # 3 int16 values (2 bytes each)
            self.unpack_format = '<hhh'  # Little-endian, 3 short integers
            self.value_names = ['ax', 'ay', 'az']
        elif sensor_choice == 2:  # IMU - 6 axes
            self.bytes_per_sample = 12  # 6 int16 values (2 bytes each)
            self.unpack_format = '<hhhhhh'  # Little-endian, 6 short integers
            self.value_names = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
        else:  # PDM - audio
            self.bytes_per_sample = 2  # 1 int16 value (2 bytes)
            self.unpack_format = '<h'  # Little-endian, 1 short integer
            self.value_names = ['audio']
        
        self.expected_samples = odr * duration
        self.total_bytes = self.expected_samples * self.bytes_per_sample
        
        # Buffer for accumulating data
        self.data_buffer = bytearray()
        self.samples_received = 0
        self.samples = []
        
        # CSV writer reference
        self.csv_writer = None
        self.csv_file = None
        
        print(f"Data collector initialized for {self.value_names}")
        print(f"Expected samples: {self.expected_samples}")
        print(f"Bytes per sample: {self.bytes_per_sample}")
        print(f"Total expected bytes: {self.total_bytes}")
    
    def set_csv_file(self, csv_filename):
        """Set up CSV output file"""
        self.csv_file = open(csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        header = self.value_names + ["date", "timestamp"]
        self.csv_writer.writerow(header)
        print(f"CSV file created: {csv_filename}")
        return self.csv_file
    
    def process_complete_samples(self):
        """Process as many complete samples as possible from the buffer"""
        samples_before = self.samples_received
        
        while len(self.data_buffer) >= self.bytes_per_sample:
            # Extract one sample
            sample_data = self.data_buffer[:self.bytes_per_sample]
            self.data_buffer = self.data_buffer[self.bytes_per_sample:]
            
            try:
                # Unpack according to sensor type
                values = struct.unpack(self.unpack_format, sample_data)
                
                # Store the sample
                self.samples.append(values)
                self.samples_received += 1
                
                # Write to CSV if writer is available
                if self.csv_writer:
                    now = datetime.datetime.now()
                    date_str = now.strftime("%Y-%m-%d")
                    time_str = now.strftime("%H:%M:%S.%f")[:-3]  # Millisecond precision
                    row = list(values) + [date_str, time_str]
                    self.csv_writer.writerow(row)
                    
                    # Flush periodically to ensure data is written
                    if self.samples_received % 100 == 0:
                        self.csv_file.flush()
                
                # Print progress periodically (every 10% of expected samples)
                if self.samples_received % max(1, self.expected_samples // 10) == 0:
                    percent = (self.samples_received / self.expected_samples) * 100
                    print(f"Progress: {percent:.1f}% - Received {self.samples_received}/{self.expected_samples} samples", end="\r")
            
            except struct.error as e:
                print(f"\nError unpacking data: {e}")
                print(f"Data bytes (hex): {sample_data.hex()}")
                # Skip this sample and try to recover
                continue
        
        # Debug: Print how many samples were processed in this call
        if self.samples_received > samples_before:
            print(f"Processed {self.samples_received - samples_before} new samples in this batch")
    
    def notification_handler(self, sender, data):
        """Handle incoming BLE notification data"""
        # Debug print the incoming data
        print(f"Received data packet: {len(data)} bytes")
        if len(data) > 0:
            print(f"First few bytes (hex): {data[:min(10, len(data))].hex()}")
        
        # Add incoming data to buffer
        self.data_buffer.extend(data)
        
        # Process complete samples
        self.process_complete_samples()
        
        # Return True if collection is complete
        if self.samples_received >= self.expected_samples:
            print(f"\nCollection complete! Received {self.samples_received} sampling items.")
            return True
        return False
    
    def cleanup(self):
        """Close files and clean up resources"""
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            print(f"CSV file closed and data saved.")

async def sampling_config(client):
    """
    Service 1: Select which sensor to sample data from.
    """
    print("\nService 1: Sensor Data Sampling")
    print("Select Sensor to sample data:")
    print("1. Kx134 (3-axes)")
    print("2. IMU (6 axes)")
    print("3. PDM (audio)")

    while True:
        try:
            choice = int(input("Select Sensor (1, 2, or 3): "))
            if choice in [1, 2, 3]:
                break
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
        except ValueError:
            print("Invalid input. Please enter a number.")
    
    if choice == 1:
        # KX-134: Three possible ODR options.
        print("\nKX-134 Sensor selected.")
        print("Select ODR for KX-134:")
        print("1. 8 kHz (max 3 seconds)")
        print("2. 5 kHz (max 6 seconds)")
        print("3. 12.8 kHz (max 2 seconds)")

        while True:
            try:
                odr_choice = int(input("Select ODR option (1, 2, or 3): "))
                if odr_choice in [1, 2, 3]:
                    break
                else:
                    print("Invalid choice. Please enter 1, 2, or 3.")
            except ValueError:
                print("Invalid input. Please enter a number.")

        if odr_choice == 1:
            odr = 8000
            max_duration = 3
        elif odr_choice == 2:
            odr = 5000
            max_duration = 6
        else:
            odr = 12800
            max_duration = 2
        sensor_name = "KX-134"
        csv_header = "ax,ay,az"
    elif choice == 2:
        sensor_name = "IMU"
        odr = 2000
        max_duration = 6
        csv_header = "ax,ay,az,gx,gy,gz"
    else:
        sensor_name = "PDM"
        odr = 5000
        max_duration = 10
        csv_header = "audio"

    while True:
        try:
            duration = int(input(f"Enter sampling duration (max {max_duration} seconds): "))
            if 1 <= duration <= max_duration:
                break
            else:
                print(f"Invalid duration. Please enter a value between 1 and {max_duration}.")
        except ValueError:
            print("Invalid input. Please enter a number.")

    print(f"\nSensor Details for {sensor_name}:")
    print(f"  Output Data Rate (ODR): {odr}")
    print(f"  Maximum Sampling Duration: {max_duration} seconds")
    print(f"  Selected Sampling Duration: {duration} seconds")
    print(f"  CSV Header: {csv_header}")

    # Create a binary packet with the sensor configuration
    # Packet format: <sensor_choice (1 byte)><odr (2 bytes)><duration (2 bytes)>
    packet = struct.pack('<BHH', choice, odr, duration)
    print(f"Sending configuration packet (hex): {packet.hex()}")
    
    try:
        # Send configuration to device
        await client.write_gatt_char(S_WRITE_CHAR, packet)
        print("Sensor configuration packet sent over S_WRITE_CHAR.")
        await asyncio.sleep(1)  # Wait for device to process
    except Exception as e:
        print(f"Failed to write sensor configuration: {e}")
        return

    print("\nSend the command to start sampling data.")
    print("Press 'Y' to start sampling or 'Q' to quit.")
    
    # Setup data collector
    data_collector = DataCollector(choice, odr, duration)
    
    # Create output file
    now = datetime.datetime.now()
    filename = os.path.join(BASE_PATH, f"{sensor_name}_{odr}_ODR_{duration}_sec_{now.strftime('%Y%m%d_%H%M%S')}.csv")
    data_collector.set_csv_file(filename)
    print(f"Data will be logged to: {filename}")
    
    # Wait for user confirmation
    while True:
        command = input("Command: ").strip().lower()
        if command == 'y':
            try:
                # Send start command (0x01 instead of 0x00)
                start_cmd = b'\x01'
                print(f"Sending start command (hex): {start_cmd.hex()}")
                await client.write_gatt_char(S_WRITE_CHAR, start_cmd)
                print("Sampling started...")
                break
            except Exception as e:
                print(f"Failed to start sampling: {e}")
                return
        elif command == 'q':
            print("Exiting...")
            return
        else:
            print("Invalid command. Please enter 'Y' to start or 'Q' to quit.")

    # Create notification handler
    def notification_callback(sender, data):
        return data_collector.notification_handler(sender, data)

    try:
        # Enable notifications
        await client.start_notify(NOTIFY_CHAR, notification_callback)
        print("Notifications started. Press Ctrl+C to stop.")
        
        # Calculate buffer time based on expected sample count
        # Give more buffer for larger datasets
        buffer_time = min(5, duration + 2)  # Dynamically adjust buffer time
        expected_time = duration + buffer_time
        print(f"Collecting data for approximately {expected_time} seconds...")
        
        # Wait loop with timeout and progress updates
        collection_start = datetime.datetime.now()
        collection_timeout = collection_start + datetime.timedelta(seconds=expected_time)
        
        while datetime.datetime.now() < collection_timeout:
            await asyncio.sleep(0.5)  # Check every half second
            elapsed = (datetime.datetime.now() - collection_start).total_seconds()
            
            # Print periodic progress update
            if elapsed % 1 < 0.1:  # Roughly every second
                print(f"Time elapsed: {elapsed:.1f}s / {expected_time}s - Received {data_collector.samples_received} samples")
            
            # If we've received all the data we expect, we can stop early
            if data_collector.samples_received >= data_collector.expected_samples:
                print("\nReceived all expected samples!")
                break
                
        elapsed = (datetime.datetime.now() - collection_start).total_seconds()
        print(f"\nCollection complete after {elapsed:.1f} seconds")
        print(f"Received {data_collector.samples_received} of {data_collector.expected_samples} expected samples")
        
    except KeyboardInterrupt:
        print("\nCollection interrupted by user.")
    except Exception as e:
        print(f"\nAn error occurred during collection: {e}")
    finally:
        # Cleanup
        try:
            await client.stop_notify(NOTIFY_CHAR)
            print("Notifications stopped.")
        except Exception as e:
            print(f"Error stopping notifications: {e}")
        
        # Close files and cleanup
        data_collector.cleanup()
        if data_collector.samples_received > 0:
            print(f"Data saved to {filename}")
        else:
            print("No data was collected.")

async def ad_disconnect(client):
    """
    Service 2: Set advertising and sleep time.
    Allows user to either send custom values or keep default (send nothing) and disconnect.
    """
    print("\nService 2: Set advertising and sleep time.")
    option = input("Press 'D' to use default (do not send any data) or 'C' to configure values: ").strip().lower()

    if option == 'd':
        print("Default option selected. No data will be sent.")
    else:
        # Prompt for custom values.
        while True:
            try:
                ad_time = int(input("Enter advertising time (0-65535 ms): "))
                sleep_time = int(input("Enter sleep time (0-65535 seconds): "))
                if not (0 <= ad_time <= 65535 and 0 <= sleep_time <= 65535):
                    print("Values must be between 0 and 65535 .")
                    continue
                break
            except ValueError:
                print("Invalid input. Please enter numeric values.")
        
        # Pack the values and send
        data = struct.pack('<HH', ad_time, sleep_time)
        print(f"Sending advertising/sleep config (hex): {data.hex()}")

        try:
            await client.write_gatt_char(AD_WRITE_CHAR, data)
            await asyncio.sleep(1)  # Wait for the command to be processed
            print("Successfully wrote advertising and sleep times to the device.")
        except Exception as e:
            print(f"Failed to write to characteristic: {e}")

async def scan_and_connect():
    """Main function to scan for and connect to BLE devices"""
    print(f"Scanning for BLE devices for {SCAN_TIMEOUT} seconds...")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)

    target_device = None
    for device in devices:
        if device.name == TARGET_NAME:
            target_device = device
            break

    if not target_device:
        print(f"No device named '{TARGET_NAME}' found.")
        return

    print(f"Found {TARGET_NAME} at {target_device.address}, attempting to connect...")

    try:
        async with BleakClient(target_device.address) as client:
            if client.is_connected:
                print(f"Connected to {TARGET_NAME}!\n")

                # print("Discovering services and characteristics...\n")
                # services = await client.get_services()
                # for service in services:
                #     print(f"Service: {service.uuid} - {service.description}")
                #     for char in service.characteristics:
                #         props = ", ".join(char.properties)
                #         print(f"  Characteristic: {char.uuid} - {char.description} (Properties: {props})")
                
                # Service selection prompt
                print("Select Service:")
                print("1. Service 1 (to select sensor and get sampled data)")
                print("2. Service 2 (to set advertising and sleep time) [DEVICE WILL GET DISCONNECTED AFTERWARDS]")

                while True:
                    try:
                        choice = int(input("Select Service (1 or 2): "))
                        if choice in [1, 2]:
                            break
                        else:
                            print("Invalid choice. Please enter 1 or 2.")
                    except ValueError:
                        print("Invalid input. Please enter a number.")

                # Handle the chosen service
                if choice == 1:
                    await sampling_config(client)
                elif choice == 2:
                    await ad_disconnect(client)
            else:
                print(f"Failed to connect to {TARGET_NAME}.")
    except Exception as e:
        print(f"Connection error: {e}")

if __name__ == "__main__":
    asyncio.run(scan_and_connect())
