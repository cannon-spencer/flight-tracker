import socket
import pickle
import struct
from random import randint
from time import sleep
import serial
import requests


def fetch_filtered_opensky_data(search_range_km):
    # University of Florida coordinates
    center_latitude = 29.6465
    center_longitude = -82.3533

    # Calculate latitude/longitude range based on search_range_km
    lat_range = search_range_km / 111  # Approximate degrees for latitude
    lon_range = search_range_km / 96.2  # Approximate degrees for longitude at 29.6465° N

    # Define bounding box
    min_latitude = center_latitude - lat_range
    max_latitude = center_latitude + lat_range
    min_longitude = center_longitude - lon_range
    max_longitude = center_longitude + lon_range

    # Construct URL with calculated bounds
    url = (f"https://opensky-network.org/api/states/all?"
           f"lamin={min_latitude}&lamax={max_latitude}&"
           f"lomin={min_longitude}&lomax={max_longitude}")

    try:
        print(f"Making API request to: {url}")
        response = requests.get(url)

        # Log HTTP status code and headers
        print(f"Response Status Code: {response.status_code}")
        print(f"Response Headers: {response.headers}")

        # Check for JSON response
        data = response.json()
        #print(f"Response JSON: {data}")  # Log the raw JSON response

        # Collect all desired fields
        aircraft_list = []
        for state in data.get("states", []):
            aircraft_info = {
                "callsign": state[1],  # Call sign
                "longitude": state[5],  # Longitude
                "latitude": state[6],  # Latitude
                "geo_altitude": state[13],  # Geometric altitude
                "velocity": state[9],  # Velocity
                "true_track": state[10],  # True track (heading)
            }
            aircraft_list.append(aircraft_info)

        print(f"Collected {len(aircraft_list)} aircraft from API response.")
        return aircraft_list

    except requests.exceptions.RequestException as e:
        print(f"An error occurred while making the API request: {e}")
        return []


def main():
    # Configure UART port
    uart_port = "/dev/ttyO4"
    baud_rate = 115200

    try:
        # Open UART connection
        uart = serial.Serial(uart_port, baud_rate)
        print(f"UART connection established on {uart_port} at {baud_rate} baud.")

        while True:
            # Fetch aircraft data within 50 km range
            aircraft_list = fetch_filtered_opensky_data(200)

            for aircraft in aircraft_list:
                # Extract and sanitize data
                longitude = aircraft['longitude'] if aircraft['longitude'] is not None else 0.0
                latitude = aircraft['latitude'] if aircraft['latitude'] is not None else 0.0
                altitude = aircraft['geo_altitude'] if aircraft['geo_altitude'] is not None else 0.0
                velocity = aircraft['velocity'] if aircraft['velocity'] is not None else 0.0
                true_track = aircraft['true_track'] if aircraft['true_track'] is not None else 0.0

                # Callsign handling
                callsign = aircraft['callsign'] if aircraft['callsign'] is not None else "N/A"
                callsign = callsign.strip()[:8]  # Ensure max 8 characters

                # Pad callsign to 8 characters with spaces
                callsign_padded = callsign.ljust(8)

                # Encode to bytes
                callsign_bytes = callsign_padded.encode('ascii', 'ignore')

                # Split into two 4-byte groups
                callsign_part1 = callsign_bytes[:4]
                callsign_part2 = callsign_bytes[4:8]

                # Unpack each part into an integer (little-endian)
                callsign_int1 = struct.unpack('<I', callsign_part1)[0]
                callsign_int2 = struct.unpack('<I', callsign_part2)[0]

                # Scale arbirtarily by 10,000 to keep precision (intepreted as int32)
                longitude_int = int(longitude * 10000)
                latitude_int = int(latitude * 10000)
                altitude_int = int(altitude * 10000)
                velocity_int = int(velocity * 10000)
                true_track_int = int(true_track * 10000)

                # Send structured data
                data = struct.pack(
                    "<IIiiiii",
                    callsign_int1,
                    callsign_int2,
                    longitude_int,
                    latitude_int,
                    altitude_int,
                    velocity_int,
                    true_track_int
                )
                uart.write(data)

                # Print the data for debugging
                print(f"Sent: callsign={callsign}, longitude={longitude}, latitude={latitude}, "
                      f"altitude={altitude}, velocity={velocity}, true_track={true_track}")

                print(f"Sending raw data: {data.hex()}")

                # short sleep to allow tiva to process data
                sleep(0.01)

            # Send the end-of-burst marker
            end_marker = struct.pack("<I", 0xFFFFFFFF)
            uart.write(end_marker)

            print("Transmission Complete!")

            # Delay before collecting new data
            sleep(10)

    except serial.SerialException as e:
        print(f"Error opening UART port: {e}")
    finally:
        if uart.is_open:
            uart.close()


if __name__ == "__main__":
    main()
