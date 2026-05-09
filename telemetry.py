from pymavlink import mavutil
import socketio
import time
import math

PC_SERVER = "https://music.heretic.icu"
sio = socketio.Client()

@sio.event
def connect():
	print('Connected to Node Server')

print("Connecting to node server...")
sio.connect("https://music.heretic.icu")


PORT = "/dev/serial0"
BAUD = 57600

master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("waiting for hearbeat...")
master.wait_heartbeat()
print("Connected to Pixhawk")

master.mav.request_data_stream_send(
	master.target_system,
	master.target_component,
	mavutil.mavlink.MAV_DATA_STREAM_ALL,
	4,
	1
)

guided = False


ALT_LOW = 2.0
ALT_HIGH = 8.0
guide_alt = ALT_LOW
SEND_INTERVAL = 2.0
last_sent = 0
took_off = False
began_circle = False

RADIUS_M = 5.0
POINTS = 36
SECONDS_PER_CIRCLE = 30


def change_guidance(master, guided, new_guide):
	if new_guide == guided: return guided
	if new_guide == True:	
		set_guided_mode(master, 'GUIDED')
	elif new_guide == False:
		set_guided_mode(master, 'LOITER')
	return new_guide
	print(guided)

def set_guided_mode(master, mode):
    # mode = "GUIDED"
    mode_id = master.mode_mapping()[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def goto_gps(master, lat, lon, alt_m):
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,  # use position only
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
current_mode = 'STABILIZE'

def take_off(master, alt):
	print('TAKING OFF!!')
	master.mav.command_long_send(
		master.target_system,
		master.target_component,
		mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
		0,
		0, 0, 0, 0,
		0, 0,
		alt  # target altitude in meters
	)
	return True

def meters_to_lat_lon_offset(north_m, east_m, origin_lat_deg):
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(origin_lat_deg)))
    return dlat, dlon

def send_gps_target(master, lat, lon, alt_m):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def get_current_position(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if msg is None:
        raise RuntimeError("No GLOBAL_POSITION_INT received")

    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0

    return lat, lon, alt

def fly_gps_circle(master):
    center_lat, center_lon, current_alt = get_current_position(master)

    delay = SECONDS_PER_CIRCLE / POINTS

    while True:
        for i in range(POINTS):
            angle = 2.0 * math.pi * i / POINTS

            north = RADIUS_M * math.cos(angle)
            east = RADIUS_M * math.sin(angle)

            dlat, dlon = meters_to_lat_lon_offset(north, east, center_lat)

            target_lat = center_lat + dlat
            target_lon = center_lon + dlon

            send_gps_target(master, target_lat, target_lon, 3.0)

            time.sleep(delay)

circle_i = 0
last_circle_send = 0

def update_gps_circle(master, center_lat, center_lon, alt_m):
	global circle_i, last_circle_send

	now = time.time()
	if now - last_circle_send < 0.5:
		return

	last_circle_send = now

	radius_m = RADIUS_M
	points = POINTS

	angle = 2.0 * math.pi * circle_i / points

	north = radius_m * math.cos(angle)
	east = radius_m * math.sin(angle)

	dlat, dlon = meters_to_lat_lon_offset(north, east, center_lat)

	target_lat = center_lat + dlat
	target_lon = center_lon + dlon

	send_gps_target(master, target_lat, target_lon, alt_m)

	circle_i = (circle_i + 1) % points
	targets = {
		'lat': target_lat,
		'lon': target_lon,
		'angle': angle
	}
	if sio.connected:
		if targets:
			try:
				sio.emit("drone_target",  targets)
			except Exception as e:
				print(e)
	


while True: 
	telemetry = {}
	msg = master.recv_match(blocking=True)
	#if msg: 
		#print(msg.get_type(), msg) 
	if msg is None: 
		continue
	msg_type = msg.get_type()
	if msg_type == "GPS_RAW_INT":
		telemetry['gps_quality'] = {
			"hdop": msg.eph / 100.0 if getattr(msg, "eph", 65535) != 65535 else None,
			"satellites": getattr(msg, "satellites_visible", None),
			"fix_type": getattr(msg, "fix_type", None),
		}
		# print(msg.to_dict())
	elif msg_type == "HEARTBEAT":
		if not msg.type == mavutil.mavlink.MAV_TYPE_GCS:
			if not mavutil.mode_string_v10(msg).startswith("Mode("):
				current_mode = mavutil.mode_string_v10(msg)
			telemetry['hb'] = {
				'mode': current_mode,
				'armed': (
				msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
			) != 0
			}
			if not telemetry['hb']['armed']: took_off = False
	elif msg_type == "GLOBAL_POSITION_INT":
		telemetry["gps"] = {
			"lat": msg.lat / 1e7,
			"lon": msg.lon / 1e7,
			"alt_m": msg.relative_alt / 1000.0,
			"vx": msg.vx / 100.0,
			"vy": msg.vy / 100.0,
			"vz": msg.vz / 100.0,
			"heading_deg": msg.hdg / 100.0
		}
		lat = msg.lat / 1e7
		lon = msg.lon / 1e7
		alt = msg.relative_alt / 1000.0
		print(f"GPS: {lat}, {lon} | Relative Alt: {alt:.2f} m")
	elif msg_type == "ATTITUDE":
		telemetry["attitude"] = {
			"roll": msg.roll,
            		"pitch": msg.pitch,
			"yaw": msg.yaw,
		        "rollspeed": msg.rollspeed,
            		"pitchspeed": msg.pitchspeed,
            		"yawspeed": msg.yawspeed
        		}
		# print(f"Roll: {msg.roll:.2f}, Pitch: {msg.pitch:.2f}, Yaw: {msg.yaw:.2f}")
	elif msg_type == "SYS_STATUS":
		telemetry["battery"] = {
	            "voltage": msg.voltage_battery / 1000.0,
	            "current": msg.current_battery / 100.0,
	            "remaining": msg.battery_remaining
		}
		voltage = msg.voltage_battery / 1000.0
		remaining = msg.battery_remaining
		# print(f"Battery: {voltage:.2f} V, Remaining: {remaining:.2f}")
	#print(telemetry)
	elif msg_type == "RC_CHANNELS":
		telemetry['rc_channels'] = {
			"chan1": msg.chan1_raw,
			"chan2": msg.chan2_raw,
			"chan3": msg.chan3_raw,
			"chan4": msg.chan4_raw,
			"chan5": msg.chan5_raw,
			"chan6": msg.chan6_raw,
			"chan7": msg.chan7_raw,
			"chan8": msg.chan8_raw,
			'airborne': took_off,
		}
		if msg.chan8_raw > 1500:
			# guide_alt = ALT_HIGH
			if current_mode == 'GUIDED':
				now = time.time()
				if now - last_sent >= SECONDS_PER_CIRCLE/POINTS:
					print('MOVING TO NEW COORDINATES')
					last_sent = now
					center_lat, center_lon, current_alt = get_current_position(master)
					update_gps_circle(master, center_lat, center_lon, 3.0)
		else: 
			circle_i = 0
			last_circle_send = 0
		# else: guide_alt = ALT_LOW
		# print(guide_alt)
		if msg.chan7_raw > 1500:
			guided = change_guidance(master, guided, True)
		else: guided = change_guidance(master, guided, False)
		if guided: 
			print('IN GUIDED MODE')
			if not took_off:
				took_off = take_off(master, guide_alt)
			# else:
			# 	now = time.time()
			# 	if now - last_sent >= SEND_INTERVAL:
			# 		print('MOVING TO COORDINATES')
			# 		goto_gps(master, 32.2542000, -110.9292000, guide_alt)
			# 		last_sent = now
		#print(msg)
	if sio.connected:
		if telemetry:
			try:
				sio.emit("drone_telemetry", telemetry)
			except Exception as e:
				print(e)
		
	#print('TELEMETRY SENT')
