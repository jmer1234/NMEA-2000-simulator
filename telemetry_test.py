#!/usr/bin/env python3
import time
import math
import random
import struct
import sys

# Safe library import handling
try:
    import can
except ImportError:
    print("Error: 'python-can' library not found.", file=sys.stderr)
    print("Please install it using: sudo apt install python3-can", file=sys.stderr)
    sys.exit(1)

# Conversions constants
KNOTS_TO_M_S = 0.514444
BOAT_SPEED_MODIFIER = 0.4

# Canyon Lake, TX (Comal County) bounding box.
# Rough box that contains the lake's main body, from the dam near
# Canyon City up to Cranes Mill / Potters Creek at the northwest end.
CANYON_LAKE_BOUNDS = {
    "north": 29.930,
    "south": 29.845,
    "east": -98.170,
    "west": -98.320,
}

# Starting point roughly in the middle of the lake
START_LAT = 29.8746
START_LON = -98.2496

def can_id_from_pgn(pgn, priority=3, source_address=1):
    """Combines PGN, priority, and source address into a 29-bit CAN ID for N2K."""
    return (priority << 26) | (pgn << 8) | source_address

def pack_rapid_gps_pgn129025(lat, lon):
    """Packs Latitude and Longitude into PGN 129025 (Position, Rapid Update)."""
    lat_raw = int(lat * 10000000) 
    lon_raw = int(lon * 10000000) 
    return struct.pack("<ii", lat_raw, lon_raw)

def pack_magnetic_variation_pgn127258(variation_rad):
    """Packs Magnetic Variation into PGN 127258."""
    sid = 0x01
    source = 0x01                              # Source: 0x01 = WMM Chart Table
    days_since_1970 = 0xFFFF                   # 0xFFFF = Data Not Available
    var_raw = int(variation_rad * 10000) & 0xFFFF 
    return struct.pack("<BBHH", sid, source, days_since_1970, var_raw) + b'\xFF\xFF'

def pack_cog_sog_pgn129026(cog_rad, sog_m_s, reference_type="true"):
    """Packs Course Over Ground (COG) and Speed Over Ground (SOG) into PGN 129026."""
    sid = 0x01                                 
    cog_ref = 0x01 if reference_type.lower() == "magnetic" else 0x00
    reserved_bits = 0x3F                       
    field2 = (cog_ref & 0x03) | (reserved_bits << 2) 
    
    cog_raw = int(cog_rad * 10000) & 0xFFFF    
    sog_raw = int(sog_m_s * 100) & 0xFFFF     
    return struct.pack("<BBHHH", sid, field2, cog_raw, sog_raw, 0xFFFF)

def pack_heading_pgn127250(heading_rad, reference_type="magnetic"):
    """Packs Vessel Heading into PGN 127250 (Vessel Heading)."""
    sid = 0x01
    hdg_raw = int(heading_rad * 10000) & 0xFFFF 
    deviation = 0x7FFF                         
    variation = 0x7FFF                         
    ref = 0x01 if reference_type.lower() == "magnetic" else 0x00
    return struct.pack("<BHHHB", sid, hdg_raw, deviation, variation, ref)

def pack_rate_of_turn_pgn127251(rot_rad_s):
    """Packs Rate of Turn into PGN 127251 (Rate of Turn)."""
    sid = 0x01
    rot_raw = int(rot_rad_s * 100000000)
    return struct.pack("<Bi", sid, rot_raw) + b'\xFF\xFF\xFF'

def pack_attitude_pgn127257(pitch_rad, roll_rad, yaw_rad=0.0):
    """Packs Pitch, Roll, and Yaw into PGN 127257 (Attitude)."""
    sid = 0x01
    yaw_raw = int(yaw_rad * 10000)     
    pitch_raw = int(pitch_rad * 10000) 
    roll_raw = int(roll_rad * 10000)   
    return struct.pack("<Bhhh", sid, yaw_raw, pitch_raw, roll_raw)

def pack_apparent_wind_pgn130306(speed_m_s, angle_rad):
    """Packs Apparent Wind Speed and Angle matching a B&G masthead sensor."""
    sid = 0x01                                 
    speed_raw = int(speed_m_s * 100) & 0xFFFF  
    angle_raw = int(angle_rad * 10000) & 0xFFFF 
    reference = 0x02                           
    return struct.pack("<BHHB", sid, speed_raw, angle_raw, reference) + b'\xFF\xFF'

def pack_depth_pgn128267(depth_meters):
    """Packs depth measurements into PGN 128267 (Water Depth)."""
    sid = 0x01
    depth_raw = int(depth_meters * 10) & 0xFFFFFFFF 
    offset = 0x0000                                 
    reserved = 0xFF                                 
    return struct.pack("<BIhB", sid, depth_raw, offset, reserved)

def pack_stw_pgn128259(stw_m_s):
    """Packs Speed Through Water into PGN 128259 (Speed)."""
    sid = 0x01
    stw_raw = int(stw_m_s * 100) & 0xFFFF  
    stw_magnetic = 0xFFFF                  
    reserved = 0xFF                        
    return struct.pack("<BHHB", sid, stw_raw, stw_magnetic, reserved) + b'\xFF\xFF'

def pack_temp_pgn130310(temp_kelvin):
    """Packs sea temperature values into PGN 130310 (Environmental Parameters)."""
    sid = 0x01
    temp_instance = 0x02 
    temp_raw = int(temp_kelvin * 100) & 0xFFFF 
    humidity = 0xFFFF                           
    pressure = 0xFFFF                           
    return struct.pack("<BBHHH", sid, temp_instance, temp_raw, humidity, pressure)

def move_position(lat, lon, bearing_rad, distance_m):
    """Applies dead reckoning to shift Lat/Lon position accurately."""
    EARTH_RADIUS = 6378137.0 
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    angular_dist = distance_m / EARTH_RADIUS
    
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(angular_dist) +
        math.cos(lat_rad) * math.sin(angular_dist) * math.cos(bearing_rad)
    )
    new_lon_rad = lon_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_dist) * math.cos(lat_rad),
        math.cos(angular_dist) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )
    
    new_lon_deg = (math.degrees(new_lon_rad) + 540) % 360 - 180
    return math.degrees(new_lat_rad), new_lon_deg

def clamp_to_lake(lat, lon, cog_true_deg, heading_velocity, bounds=CANYON_LAKE_BOUNDS):
    """
    Keeps the simulated position inside the Canyon Lake bounding box.
    If the boat drifts past a bound, it's pulled back inside and its
    course is reflected so it heads back toward open water instead of
    running aground.
    """
    bounced = False

    if lat > bounds["north"]:
        lat = bounds["north"]
        cog_true_deg = (-cog_true_deg) % 360   # reflect north/south component
        bounced = True
    elif lat < bounds["south"]:
        lat = bounds["south"]
        cog_true_deg = (-cog_true_deg) % 360
        bounced = True

    if lon > bounds["east"]:
        lon = bounds["east"]
        cog_true_deg = (180 - cog_true_deg) % 360  # reflect east/west component
        bounced = True
    elif lon < bounds["west"]:
        lon = bounds["west"]
        cog_true_deg = (180 - cog_true_deg) % 360
        bounced = True

    if bounced:
        heading_velocity = 0.0

    return lat, lon, cog_true_deg, heading_velocity

def run_simulation(bus):
    """Handles the main loop updates sequentially without layout nesting."""
    # Simulation Initial baselines
    wind_speed_knots = 12.0                     
    current_depth = 12.4                       
    water_temp_c = 18.5                        
    MAGNETIC_VARIATION = 4.0                   
    
    lat, lon = START_LAT, START_LON
    cog_true_deg = 45.0                             
    awa_deg = 30.0                             
    
    counter = 0

    print("[-] Engine Active. Press Ctrl+C to terminate.")
    print(f"[-] Simulated vessel bounded to Canyon Lake, TX: "
          f"N {CANYON_LAKE_BOUNDS['north']}, S {CANYON_LAKE_BOUNDS['south']}, "
          f"E {CANYON_LAKE_BOUNDS['east']}, W {CANYON_LAKE_BOUNDS['west']}")

    # NEW: Tracks the current turning momentum of the boat
    heading_velocity = 0.0 

    while True:
        # 1. Update Dynamic Environmental Parameters
        wind_speed_knots += random.uniform(-0.25, 0.25)
        wind_speed_knots = max(8.0, min(18.0, wind_speed_knots))
        wind_speed_m_s = wind_speed_knots * KNOTS_TO_M_S

        current_depth += random.uniform(-0.1, 0.1)
        current_depth = max(2.0, min(40.0, current_depth))

        boat_speed_knots = wind_speed_knots * BOAT_SPEED_MODIFIER
        boat_speed_m_s = boat_speed_knots * KNOTS_TO_M_S

        # 2. Process High Frequency Heading and GPS Math (10Hz Cycle Slice)
        # Nudge the turning velocity slightly on every tick
        heading_velocity += random.uniform(-0.05, 0.05)
        
        # Keep the turn rate gentle (max 1.5 degrees per second in either direction)
        heading_velocity = max(-0.15, min(0.15, heading_velocity))
        
        # Apply a tiny "drag" factor so the boat naturally wants to straighten out eventually
        heading_velocity *= 0.98

        # Update the true heading by our current turning momentum
        cog_true_deg = (cog_true_deg + heading_velocity) % 360
        cog_true_rad = math.radians(cog_true_deg)
        
        cog_mag_deg = (cog_true_deg - MAGNETIC_VARIATION) % 360
        cog_mag_rad = math.radians(cog_mag_deg)
        
        lat, lon = move_position(lat, lon, cog_true_rad, boat_speed_m_s / 10.0)

        # Keep the vessel within the Canyon Lake, TX bounding box
        lat, lon, cog_true_deg, heading_velocity = clamp_to_lake(
            lat, lon, cog_true_deg, heading_velocity
        )
        cog_true_rad = math.radians(cog_true_deg)
        cog_mag_deg = (cog_true_deg - MAGNETIC_VARIATION) % 360
        cog_mag_rad = math.radians(cog_mag_deg)
        
        # RESTORED: Wave motion math required for Section 3
        pitch_sim = math.radians(1.5 * math.sin(time.time() * 2))
        roll_sim = math.radians(3.0 * math.cos(time.time() * 1.5))
        rot_sim = 0.05 * math.cos(time.time())

        # 3. Compile and Send High Speed 10Hz Datagrams
        rapid_gps_p = pack_rapid_gps_pgn129025(lat, lon)
        cog_true_p = pack_cog_sog_pgn129026(cog_true_rad, boat_speed_m_s, reference_type="true")
        cog_mag_p = pack_cog_sog_pgn129026(cog_mag_rad, boat_speed_m_s, reference_type="magnetic")
        h2183_hdg_p = pack_heading_pgn127250(cog_mag_rad, reference_type="magnetic")
        h2183_att_p = pack_attitude_pgn127257(pitch_sim, roll_sim)
        h2183_rot_p = pack_rate_of_turn_pgn127251(rot_sim)

        bus.send(can.Message(arbitration_id=can_id_from_pgn(129025, 2, 10), data=rapid_gps_p, is_extended_id=True))
        bus.send(can.Message(arbitration_id=can_id_from_pgn(129026, 2, 10), data=cog_true_p, is_extended_id=True))
        bus.send(can.Message(arbitration_id=can_id_from_pgn(129026, 2, 10), data=cog_mag_p, is_extended_id=True))
        bus.send(can.Message(arbitration_id=can_id_from_pgn(127250, 2, 42), data=h2183_hdg_p, is_extended_id=True))
        bus.send(can.Message(arbitration_id=can_id_from_pgn(127257, 3, 42), data=h2183_att_p, is_extended_id=True))
        bus.send(can.Message(arbitration_id=can_id_from_pgn(127251, 2, 42), data=h2183_rot_p, is_extended_id=True))

        # 4. Process Slower 1Hz Datagram Packages (Every 10 Counts)
        if counter % 10 == 0:
            var_p = pack_magnetic_variation_pgn127258(math.radians(MAGNETIC_VARIATION))
            bus.send(can.Message(arbitration_id=can_id_from_pgn(127258, 3, 10), data=var_p, is_extended_id=True))

            awa_deg = (awa_deg + random.uniform(-2.0, 2.0)) % 360
            wind_p = pack_apparent_wind_pgn130306(wind_speed_m_s, math.radians(awa_deg))
            bus.send(can.Message(arbitration_id=can_id_from_pgn(130306, 2, 24), data=wind_p, is_extended_id=True))

            depth_p = pack_depth_pgn128267(current_depth)
            stw_p = pack_stw_pgn128259(boat_speed_m_s)
            temp_p = pack_temp_pgn130310(water_temp_c + 273.15)

            bus.send(can.Message(arbitration_id=can_id_from_pgn(128267, 3, 35), data=depth_p, is_extended_id=True))
            bus.send(can.Message(arbitration_id=can_id_from_pgn(128259, 2, 35), data=stw_p, is_extended_id=True))
            bus.send(can.Message(arbitration_id=can_id_from_pgn(130310, 5, 35), data=temp_p, is_extended_id=True))

            print(f"[SIM] Wind: {wind_speed_knots:.2f}kt | Boat Spd: {boat_speed_knots:.2f}kt | Depth: {current_depth:.1f}m | Pos: {lat:.4f},{lon:.4f}")

        counter += 1
        time.slice_step = 0.1
        time.sleep(0.1)

def main():
    print("[-] Connecting to Waveshare hardware interface can1...")
    bus = None
    try:
        bus = can.interface.Bus(channel='can1', interface='socketcan')
        print("[-] Hardware layer established successfully.")
        run_simulation(bus)
    except KeyboardInterrupt:
        print("\n[-] Simulation interrupted manually.")
    except Exception as e:
        print(f"[!] Critical Error Encountered: {e}", file=sys.stderr)
    finally:
        if bus is not None:
            bus.shutdown()
        print("[-] Powering down interface...")

if __name__ == "__main__":
    main()
