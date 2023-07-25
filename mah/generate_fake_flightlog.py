from pymavlink import mavutil
import time

mavutil.set_dialect("common")

master = mavutil.mavlogfile(
    "logfile.tlog", source_system=1, source_component=1, write=True
)

master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    1,
    1,
)

start = time.time()
alt = 900
lat = 47.12805502756898
lon = 15.212404260012544
deltag = 0.0001

for i in range(100):
    master.mav.gps_input_send(
        int(start + i * 10),  # Timestamp (micros since boot or Unix epoch)
        0,  # ID of the GPS for multiple GPS inputs
        # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
        # All other fields must be provided.
        8 | 16 | 32,
        0,  # GPS time (milliseconds from start of GPS week)
        0,  # GPS week number
        3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        int((lat + deltag * i) * 1e7),  # Latitude (WGS84), in degrees * 1E7
        int((lon + deltag * i) * 1e7),  # Longitude (WGS84), in degrees * 1E7
        850,  # Altitude (AMSL, not WGS84), in m (positive for up)
        1,  # GPS HDOP horizontal dilution of position in m
        1,  # GPS VDOP vertical dilution of position in m
        0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
        0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        0,  # GPS speed accuracy in m/s
        0,  # GPS horizontal accuracy in m
        0,  # GPS vertical accuracy in m
        7,  # Number of satellites visible.
    )
    master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            i, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
