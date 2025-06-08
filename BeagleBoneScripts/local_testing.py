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
        # Make a GET request to the OpenSky Network API with bounding box filters
        response = requests.get(url)
        response.raise_for_status()  # Check for HTTP errors

        # Parse the JSON response
        data = response.json()

        # Filter and print only the desired fields
        print("Filtered Flight Data from OpenSky Network:")
        for state in data.get("states", []):
            aircraft_info = {
                "icao24": state[0],  # ICAO24 identifier
                "callsign": state[1],  # Call sign
                "longitude": state[5],  # Longitude
                "latitude": state[6],  # Latitude
                "geo_altitude": state[13],  # Geometric altitude
                "on_ground": state[8],  # On ground status
                "velocity": state[9],  # Velocity
                "true_track": state[10],  # True track (heading)
            }

            # Print each filtered aircraft info
            print(aircraft_info)

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")


# Example usage with a 250 km search range
fetch_filtered_opensky_data(100)
