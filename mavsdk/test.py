import asyncio
import math
from mavsdk import System

# Reference location
latitude = -29.78634556614431
longitude = -51.16386383414946
altitude = 10  # Desired altitude in meters
radius = 50  # Circle radius in meters
tolerance = 5  # Tolerance in meters for reaching a waypoint

async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")  # Adjust the address if needed

    # Wait for the drone to connect
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Set the mode to "Offboard" and arm the drone
    print("Arming the drone...")
    await drone.action.arm()

    # Start publishing setpoints before taking off
    asyncio.ensure_future(keep_sending_setpoints(drone))

    # Take off
    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(10)  # Wait for the drone to reach takeoff altitude

    # Calculate the circular path
    num_points = 36  # Number of points on the circle (e.g., one point every 10 degrees)
    angle_increment = 2 * math.pi / num_points

    # Function to get the current position
    async def get_current_position():
        async for position in drone.telemetry.position():
            return position.latitude_deg, position.longitude_deg, position.absolute_altitude_m

    # Move the drone in a circular path
    print("Flying in a circle...")
    for i in range(num_points + 1):  # Loop through each point on the circle
        angle = i * angle_increment
        delta_lat = (radius / 6378137) * (math.cos(angle))  # Earth's radius in meters
        delta_lon = (radius / 6378137) * (math.sin(angle) / math.cos(math.radians(latitude)))

        new_latitude = latitude + math.degrees(delta_lat)
        new_longitude = longitude + math.degrees(delta_lon)

        print(f"Moving to: Latitude: {new_latitude}, Longitude: {new_longitude}")
        await drone.action.goto_location(new_latitude, new_longitude, altitude, 0)

        # Wait until the UAV reaches the waypoint
        while True:
            current_latitude, current_longitude, _ = await get_current_position()
            distance = calculate_distance(current_latitude, current_longitude, new_latitude, new_longitude)
            if distance < tolerance:
                print(f"Reached waypoint: Latitude: {new_latitude}, Longitude: {new_longitude}")
                break
            await asyncio.sleep(1)  # Check position every second

    # Return to launch
    print("Returning to launch...")
    await drone.action.return_to_launch()

# Helper function to continuously send setpoints
async def keep_sending_setpoints(drone):
    while True:
        await drone.action.set_takeoff_altitude(altitude)
        await asyncio.sleep(0.5)  # Send setpoints every 0.5 seconds

# Helper function to calculate the distance between two lat/lon points
def calculate_distance(lat1, lon1, lat2, lon2):
    # Haversine formula to calculate distance
    r = 6378137  # Earth's radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c

if __name__ == "__main__":
    asyncio.run(run())
