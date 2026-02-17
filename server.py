from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import random
import time
import math

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

# ========================================
# GLOBAL STATE
# ========================================
drone_active = True
predicted_path = []
mission_alerts = []
return_home_active = False
gps_home_position = None
mission_destination = None
actual_positions = []  # list of emitted actual positions (for RTH)
mission_index = 0      # index into predicted_path where drone is/was

@app.route('/')
def index():
    return render_template('index.html')

# ========================================
# 1. PREDICTED MISSION PATH GENERATOR
# ========================================
def interpolate_waypoints(start, end, num_points):
    """
    Linear interpolation between two waypoints.
    Returns list of dicts with x, y, z.
    """
    points = []
    for i in range(num_points):
        t = i / (num_points - 1) if num_points > 1 else 0
        point = {
            'x': start['x'] + t * (end['x'] - start['x']),
            'y': start['y'] + t * (end['y'] - start['y']),
            'z': start['z'] + t * (end['z'] - start['z'])
        }
        points.append(point)
    return points

def generate_predicted_path():
    """
    Generate a deterministic surveillance grid patrol route.
    
    Waypoints form rectangular zig-zag pattern:
    - Segment A: Move East (X increases)
    - Segment B: Move North (Y increases)
    - Segment C: Move West (X decreases)
    - Segment D: Move South (Y decreases)
    - Segment E: Move East to final checkpoint
    
    Total: 180-220 points with smooth interpolation.
    """
    # Use real-world GPS anchors (lat, lon, alt) converted to ECEF
    # Home: Thapar CSED Department
    HOME_LAT = 30.3558
    HOME_LON = 76.3650
    HOME_ALT = 250.0

    # Destination: Hostel M
    DEST_LAT = 30.3581
    DEST_LON = 76.3598
    DEST_ALT = 250.0

    # helper: geodetic -> ECEF (WGS84)
    def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
        a = 6378137.0
        e2 = 6.69437999014e-3
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        N = a / math.sqrt(1 - e2 * (math.sin(lat)**2))
        x = (N + alt_m) * math.cos(lat) * math.cos(lon)
        y = (N + alt_m) * math.cos(lat) * math.sin(lon)
        z = ((1 - e2) * N + alt_m) * math.sin(lat)
        return {'x': x, 'y': y, 'z': z}

    global gps_home_position, mission_destination
    gps_home_position = geodetic_to_ecef(HOME_LAT, HOME_LON, HOME_ALT)
    mission_destination = geodetic_to_ecef(DEST_LAT, DEST_LON, DEST_ALT)

    # Generate smooth interpolated path between home and destination in ECEF
    num_points = 200
    path = interpolate_waypoints(gps_home_position, mission_destination, num_points)
    return path

# ========================================
# 2. SAFETY CONSTRAINTS CHECKER
# ========================================
def check_safety(actual_x, actual_y, actual_z):
    """
    Check safety constraints:
    - Boundary limit: X < 500, Y < 500
    - Min altitude: Z > 50
    - Max altitude: Z < 300
    Returns dict with boolean results
    """
    boundary_ok = actual_x < 500 and actual_y < 500
    altitude_ok = actual_z > 50 and actual_z < 300
    
    return {
        'boundary_ok': boundary_ok,
        'altitude_ok': altitude_ok
    }

# ========================================
# 3. MATHEMATICAL DEBUG CALCULATOR
# ========================================
def compute_math_debug(actual_x, actual_y, actual_z, pred_idx):
    """
    Compute Euclidean deviation and constraint checks.
    Uses current predicted point based on progress through path.
    """
    if pred_idx >= len(predicted_path):
        pred_idx = len(predicted_path) - 1
    
    pred = predicted_path[pred_idx]
    px, py, pz = pred['x'], pred['y'], pred['z']
    
    # Euclidean deviation
    deviation = math.sqrt(
        (actual_x - px)**2 + 
        (actual_y - py)**2 + 
        (actual_z - pz)**2
    )
    
    # Safety checks
    safety = check_safety(actual_x, actual_y, actual_z)
    deviation_ok = deviation < 40
    
    # distances to home and destination if available
    dist_home = None
    dist_dest = None
    global gps_home_position, mission_destination
    if gps_home_position is not None:
        dx = actual_x - gps_home_position['x']
        dy = actual_y - gps_home_position['y']
        dz = actual_z - gps_home_position['z']
        dist_home = round(math.sqrt(dx*dx + dy*dy + dz*dz), 2)
    if mission_destination is not None:
        dx2 = actual_x - mission_destination['x']
        dy2 = actual_y - mission_destination['y']
        dz2 = actual_z - mission_destination['z']
        dist_dest = round(math.sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2), 2)

    return {
        'actual': {'x': round(actual_x, 2), 'y': round(actual_y, 2), 'z': round(actual_z, 2)},
        'predicted': {'x': round(px, 2), 'y': round(py, 2), 'z': round(pz, 2)},
        'deviation': round(deviation, 2),
        'boundary_ok': safety['boundary_ok'],
        'altitude_ok': safety['altitude_ok'],
        'deviation_ok': deviation_ok,
        'gps_home_position': gps_home_position,
        'mission_destination': mission_destination,
        'distance_from_home': dist_home,
        'distance_to_destination': dist_dest
    }


def compute_distance_from_home(x, y, z):
    """Return Euclidean distance from gps_home_position if available, else None."""
    global gps_home_position
    if gps_home_position is None:
        return None
    dx = x - gps_home_position['x']
    dy = y - gps_home_position['y']
    dz = z - gps_home_position['z']
    return round(math.sqrt(dx*dx + dy*dy + dz*dz), 2)

# ========================================
# 4. DRONE MOVEMENT SIMULATOR
# ========================================
def _interpolate_points(a, b, steps=8):
    """Return list of points interpolated between a and b (inclusive of endpoints)."""
    pts = []
    for i in range(steps):
        t = i / (steps - 1) if steps > 1 else 0
        pts.append({
            'x': a['x'] + t * (b['x'] - a['x']),
            'y': a['y'] + t * (b['y'] - a['y']),
            'z': a['z'] + t * (b['z'] - a['z']),
        })
    return pts


def capture_gps_origin(x, y, z):
    """Capture the first emitted position as GPS home origin."""
    global gps_home_position
    if gps_home_position is None:
        gps_home_position = {'x': x, 'y': y, 'z': z}
        socketio.emit('alert', {'type': 'GPS HOME SET', 'message': f'GPS home captured at X={x:.2f},Y={y:.2f},Z={z:.2f}'})


def handle_restart():
    """Resume mission from the same mission_index where it was stopped."""
    global drone_active, return_home_active
    return_home_active = False
    drone_active = True
    socketio.emit('alert', {'type': 'DRONE RESTARTED', 'message': 'Drone restarted by operator'})


def handle_return_home():
    """Trigger return-to-home: stop normal mission and retrace actual positions."""
    global return_home_active, drone_active
    drone_active = False
    return_home_active = True
    socketio.emit('alert', {'type': 'RTH INITIATED', 'message': 'Return-to-home initiated by operator'})


def simulate_drone_movement():
    """
    Main movement loop runs forever in a background thread.
    Priority:
      1) If return_home_active: follow reverse of actual_positions (RTH)
      2) Elif drone_active: follow predicted_path from mission_index
      3) Else: hold (sleep)
    
    Deviation model:
      - Maintain noise_x, noise_y, noise_z state variables
      - Update gradually: noise += random.uniform(-0.5, 0.5) per frame
      - Apply soft damping: noise *= 0.95 (or 0.97 for z)
      - Clamp to phase-based limits: SAFE ±3m, WARNING ±15m, UNSAFE ±60m
    """
    global drone_active, predicted_path, mission_alerts, actual_positions, gps_home_position, mission_index, return_home_active

    # Noise state variables for smooth deviation
    noise_x = 0.0
    noise_y = 0.0
    noise_z = 0.0

    while True:
        # 1) Return-to-home active: retrace actual_positions in reverse
        if return_home_active:
            if len(actual_positions) < 2:
                # Nothing to retrace
                socketio.emit('alert', {'type': 'DRONE RETURNED HOME', 'message': 'No flown path to return along'})
                return_home_active = False
                drone_active = False
                time.sleep(0.5)
                continue

            reverse_path = list(reversed(actual_positions))
            # Walk reverse_path with interpolation between points
            for i in range(len(reverse_path) - 1):
                if not return_home_active:
                    break
                seg = _interpolate_points(reverse_path[i], reverse_path[i+1], steps=8)
                for p in seg:
                    # Emit position and debug
                    socketio.emit('position', {'x': p['x'], 'y': p['y'], 'z': p['z']})
                    # capture home if not set
                    if gps_home_position is None:
                        capture_gps_origin(p['x'], p['y'], p['z'])
                    # compute debug and include distance from home
                    debug = compute_math_debug(p['x'], p['y'], p['z'], mission_index if mission_index < len(predicted_path) else len(predicted_path)-1)
                    debug['gps_home_position'] = gps_home_position
                    debug['distance_from_home'] = compute_distance_from_home(p['x'], p['y'], p['z'])
                    socketio.emit('math_debug', debug)
                    time.sleep(0.15)

            # Finished return home
            return_home_active = False
            drone_active = False
            socketio.emit('alert', {'type': 'DRONE RETURNED HOME', 'message': 'Drone returned to home position'})
            time.sleep(0.5)
            continue

        # 2) Normal mission execution
        if drone_active and mission_index < len(predicted_path):
            # get current waypoint
            pred = predicted_path[mission_index]
            px, py, pz = pred['x'], pred['y'], pred['z']

            # Determine phase based on mission progress
            phase_safe_end = int(len(predicted_path) * 0.4)
            phase_warning_end = int(len(predicted_path) * 0.7)
            
            if mission_index < phase_safe_end:
                phase = 'SAFE'
                max_deviation = 6.0  # ±6m
            elif mission_index < phase_warning_end:
                phase = 'WARNING'
                max_deviation = 25.0  # ±25m
            else:
                phase = 'UNSAFE'
                max_deviation = 80.0  # ±80m

            # Update noise variables gradually with larger increments
            noise_x += random.uniform(-1.2, 1.2)
            noise_y += random.uniform(-1.2, 1.2)
            noise_z += random.uniform(-0.5, 0.5)

            # Apply soft damping to prevent exponential growth
            noise_x *= 0.96
            noise_y *= 0.96
            noise_z *= 0.98

            # Clamp noise to phase-based limits
            noise_x = max(min(noise_x, max_deviation), -max_deviation)
            noise_y = max(min(noise_y, max_deviation), -max_deviation)
            noise_z = max(min(noise_z, max_deviation), -max_deviation)

            # Apply noise to predicted position
            x = px + noise_x
            y = py + noise_y
            z = pz + noise_z

            # emit position
            socketio.emit('position', {'x': x, 'y': y, 'z': z})

            # capture gps home if unset
            if gps_home_position is None:
                capture_gps_origin(x, y, z)

            # store actual position for RTH
            actual_positions.append({'x': x, 'y': y, 'z': z})

            # compute math debug and include home distance
            debug = compute_math_debug(x, y, z, mission_index)
            debug['gps_home_position'] = gps_home_position
            debug['distance_from_home'] = compute_distance_from_home(x, y, z)
            socketio.emit('math_debug', debug)

            # check safety and emit alerts (detection only)
            safety = check_safety(x, y, z)
            if not safety['boundary_ok']:
                key = f'BOUNDARY_{mission_index // 10}'
                if key not in mission_alerts:
                    mission_alerts.append(key)
                    socketio.emit('alert', {'type': 'BOUNDARY VIOLATION', 'message': f'Boundary exceeded at idx {mission_index}: X={x:.1f},Y={y:.1f}'})
            if not safety['altitude_ok']:
                key = f'ALT_{mission_index // 10}'
                if key not in mission_alerts:
                    mission_alerts.append(key)
                    socketio.emit('alert', {'type': 'ALTITUDE LIMIT EXCEEDED', 'message': f'Altitude out of range at idx {mission_index}: Z={z:.1f}'})
            if not debug['deviation_ok']:
                key = f'DEV_{mission_index // 10}'
                if key not in mission_alerts:
                    mission_alerts.append(key)
                    socketio.emit('alert', {'type': 'PATH DEVIATION HIGH', 'message': f'Deviation {debug["deviation"]}m at idx {mission_index}'})

            # phase update
            socketio.emit('phase_update', {'phase': phase, 'position': mission_index, 'total': len(predicted_path)})

            mission_index += 1
            time.sleep(0.3)
            continue

        # 3) Hold position (drone inactive)
        time.sleep(0.3)


# ========================================
# SOCKETIO EVENT HANDLERS
# ========================================
@socketio.on('connect')
def send_predicted_path():
    print("Client connected — sending predicted path")
    socketio.emit('predicted_path', predicted_path)


@socketio.on('stop_drone')
def handle_stop_drone():
    """Emergency stop handler"""
    global drone_active
    drone_active = False
    socketio.emit('alert', {'type': 'DRONE STOPPED BY OPERATOR', 'message': 'Drone has been stopped by operator'})
    print("Drone stopped by operator")


@socketio.on('restart_drone')
def socket_restart_drone():
    handle_restart()


@socketio.on('return_home')
def socket_return_home():
    handle_return_home()
# ========================================
# INITIALIZATION
# ========================================
predicted_path = generate_predicted_path()
print(f"Generated predicted path with {len(predicted_path)} points")

# Start drone simulation
threading.Thread(target=simulate_drone_movement, daemon=True).start()

if __name__ == '__main__':
    print("Server Starting...")
    socketio.run(app, host="127.0.0.1", port=5050)
