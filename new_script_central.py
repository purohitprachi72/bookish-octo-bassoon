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
    # exit(1)
today = datetime.datetime.now().strftime("%d-%m-%Y")
folder_name = f"nRF52S_SensorData_{today}"
BASE_PATH = os.path.join(desktop, folder_name)
if not os.path.exists(BASE_PATH):
    os.makedirs(BASE_PATH)
    print(f"Created directory: {BASE_PATH}")
else:
    print(f"Directory already exists: {BASE_PATH}")

async def sampling_config(client):
    """
    Service 1: Select which sensor to sample data from.
    """
    print("\nService 1: Sensor Data Sampling")
    print("Select Sensor to sample data:")
    print("1. Kx134 (3-axes)")
    print("2. IMU (6 axes)")
    print("3. PDM (audio)")

    while client.is_connected:
        try:
            choice = int(input("Select Sensor (1, 2, or 3): "))
            if choice in [1, 2, 3]:
                break
            else:
                print("Invalid choice. Please enter 1, 2, or 3.")
        except ValueError:
            print("Invalid input. Please enter a number.")

    # try:
    #     # Convert choice to bytes (1 byte for the sensor selection).
    #     data = struct.pack('<B', choice)
    #     await client.write_gatt_char(S_WRITE_CHAR, data)
    #     await asyncio.sleep(1)  # Wait for the command to be processed
    #     print("Successfully wrote sensor selection to the device.")
    # except Exception as e:
    #     print(f"Failed to write to characteristic: {e}")
    #     return
    
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

    try:
        packet = struct.pack('<BHH', choice, odr, duration)
        await client.write_gatt_char(S_WRITE_CHAR, packet)
        await asyncio.sleep(1)
        print("Sensor configuration packet sent over S_WRITE_CHAR.")
    except Exception as e:
        print(f"Failed to write sensor configuration: {e}")
        return

    print("\nSend the command to start sampling data.")
    print("Press 'Y' to start sampling or 'Q' to quit.")
    while True:
        command = input("Command: ").strip().lower()
        if command == 'y':
            try:
                await client.write_gatt_char(S_WRITE_CHAR, b'\x01')
                print("Sampling started.")
                break
            except Exception as e:
                print(f"Failed to start sampling: {e}")
                return
        elif command == 'q':
            print("Exiting...")
            return
        else:
            print("Invalid command. Please enter 'Y' to start or 'Q' to quit.")

    now = datetime.datetime.now()
    # filename = f"{sensor_name}_{odr}_ODR_{duration}_sec_{now.strftime('%Y%m%d_%H%M%S')}.csv"
    filename = os.path.join(BASE_PATH, f"{sensor_name}_{odr}_ODR_{duration}_sec_{now.strftime('%Y%m%d_%H%M%S')}.csv")
    csvfile = open(filename, 'w', newline='')
    writer = csv.writer(csvfile)
    header = csv_header.split(",") + ["date", "timestamp"]
    writer.writerow(header)
    print(f"Logging data to CSV file: {filename}")

    def notification_handler(sender, data):
        now2 = datetime.datetime.now()
        date_str = now2.strftime("%Y-%m-%d")
        time_str = now2.strftime("%H:%M:%S")
        try:
            if choice == 1:
                values = struct.unpack('<hhh', data)
                # ax, ay, az = values
            elif choice == 2:
                values = struct.unpack('<hhhhhh', data)
                # ax, ay, az, gx, gy, gz = values
            else:
                values = struct.unpack('<h', data)
                # audio = values[0]
        except Exception as e:
            values = (data.hex(),)
        row = list(values) + [date_str, time_str]
        # writer.writerow(list(values) + [date_str, time_str])
        try:
            writer.writerow(row)
        except Exception as e:
            writer.writerrow([data.hex()] + [date_str, time_str])
        csvfile.flush()

    try:
        await client.start_notify(NOTIFY_CHAR, notification_handler)
        print("Notifications started. Press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1)  # Keep the loop running to receive notifications
    except KeyboardInterrupt:
        print("\nStopping notifications...")
        # await client.stop_notify(NOTIFY_CHAR)
        # csvfile.close()
        # print("Notifications stopped and CSV file closed.")
    except Exception as e:
        print(f"An error occurred: {e}")
        # await client.stop_notify(NOTIFY_CHAR)
        # csvfile.close()
        # print("Notifications stopped and CSV file closed.")
    finally:
        await client.stop_notify(NOTIFY_CHAR)
        # csvfile.flush()
        csvfile.close()
        print(f"Data saved to {filename}.")
        await client.disconnect()
        await asyncio.sleep(1)  # Wait for the disconnect to complete
        print("Disconnected from the device.")

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
        while client.is_connected:
            try:
                # ad_time = int(input("Enter advertising time (in ms): "))
                # sleep_time = int(input("Enter sleep time (in seconds): "))
                ad_time = int(input("Enter advertising time (0-65535 ms): "))
                sleep_time = int(input("Enter sleep time (0-65535 seconds): "))
                if not (0 <= ad_time <= 65535 and 0 <= sleep_time <= 65535):
                    print("Values must be between 0 and 65535 .")
                    continue
                break
            except ValueError:
                print("Invalid input. Please enter numeric values.")
        
        # Pack the two integer values into bytes (little-endian unsigned integers).
        # Pack the two integer values into bytes (unsigned 8-bit each)
        data = struct.pack('<HH', ad_time, sleep_time)

        try:
            await client.write_gatt_char(AD_WRITE_CHAR, data)
            await asyncio.sleep(1)  # Wait for the command to be processed
            print("Successfully wrote advertising and sleep times to the device.")
        except Exception as e:
            print(f"Failed to write to characteristic: {e}")

    # Disconnect from the device and exit.
    # await client.disconnect()
    # print("Device disconnected.")
    # sys.exit(0)

async def scan_and_connect():
    print(f"Scanning for BLE devices for {SCAN_TIMEOUT} seconds...")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)

    target_device = None
    for device in devices:
        # print(f"Found device: {device.name} ({device.address})")
        if device.name == TARGET_NAME:
            target_device = device
            break

    if not target_device:
        print(f"No device named '{TARGET_NAME}' found.")
        return

    print(f"Found {TARGET_NAME} at {target_device.address}, attempting to connect...")

    async with BleakClient(target_device.address) as client:
        if client.is_connected:
            print(f"Connected to {TARGET_NAME}!\n")

            print("Discovering services and characteristics...\n")
            services = await client.get_services()
            for service in services:
                print(f"Service: {service.uuid} - {service.description}")
                for char in service.characteristics:
                    props = ", ".join(char.properties)
                    print(f"  Characteristic: {char.uuid} - {char.description} (Properties: {props})")
            

            # Service selection prompt
            print("Select Service:")
            print("1. Service 1 (to select sensor and get sampled data)")
            print("2. Service 2 (to set advertising and sleep time) [DEVICE WILL GET DISCONNECTED AFTERWARDS]")

            while client.is_connected:
                try:
                    choice = int(input("Select Service (1 or 2): "))
                    if choice in [1, 2]:
                        break
                    else:
                        print("Invalid choice. Please enter 1 or 2.")
                except ValueError:
                    print("Invalid input. Please enter a number.")

            # Continue with handling the chosen service...
            if choice == 1:
                await sampling_config(client)
            elif choice == 2:
                await ad_disconnect(client)
                # print("Disconnecting from device.")

        else:
            print(f"Failed to connect to {TARGET_NAME}.")

if __name__ == "__main__":
    asyncio.run(scan_and_connect())