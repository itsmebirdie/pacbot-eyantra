"""PacBot Task 2A - FAST variant of the solver (same structure and safety logic;
quicker turn/stop endings, a slightly higher cruise speed and a planner that
minimises time, so it prefers fewer turns). The original is task_2a_boilerplate.py.

PacBot Task 2A - solver boilerplate.

Fill in the parts marked TODO. Everything else - the MQTT connection, the
message formats, the maze layout and the robot's dimensions - is done for
you and matches the simulator.

The task: the robot spawns in a random cell facing a random open direction.
Drive through the pellet's cell, then leave the maze through the exit on
the -y (south) side of cell (14, 14). Score: 20 for solving + 10 for the pellet
- 5 per wall collision.

What you get over MQTT:
  pacbot/poses    once at startup (retained): the robot's spawn pose
                  -> Solver.on_poses()
  pacbot/pellets  once at startup (retained): the pellet's pose
                  -> Solver.on_pellets()
  pacbot/sensors  every physics step (10 ms): ToF distances, gyro,
                  accelerometer, wheel encoder ticks -> Solver.step()
  pacbot/result   once, when the run ends: {"solved", "collisions"}
What you send:
  pacbot/wheel_vel  wheel speeds in rad/s - the return value of Solver.step()

As given, this file connects, reads the poses and the pellet, and keeps the
robot still.

Run:
    ./task_2a_launch                 # terminal 1
    python3 task_2a_fast.py          # terminal 2
"""
import heapq
import json
import math

import paho.mqtt.client as mqtt

# ------------------------------------------------------------------ MQTT
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # sim -> us, every physics step
TOPIC_POSES = "pacbot/poses"          # sim -> us, once, retained: robot spawn pose
TOPIC_PELLETS = "pacbot/pellets"      # sim -> us, once, retained: pellet pose
TOPIC_RESULT = "pacbot/result"        # sim -> us, once, when the run ends
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # us -> sim

# ------------------------------------------------------------------ maze
N = 15            # cells per side
PITCH = 0.22      # m, cell centre to cell centre
WALL_HALF = 0.005  # m, half the wall thickness
# Cells are (row, col), as the maze drawing and pacbot/poses use: row 0 at
# the top, column 0 at the left. As in Task 2B, cell (row, col) is centred
# at world (col * PITCH, (N - 1 - row) * PITCH) - see cell_centre().
EXIT = (N - 1, N - 1, 3)  # the only opening in the border: -y side of (14, 14)

# Directions: 0 = +x, 1 = +y, 2 = -x, 3 = -y (the numbering pacbot/poses uses).
# STEP[d] is the (d_row, d_col) of one cell in direction d: +x is the next
# column, +y the row above.
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

# Copy of kLayout in maze_2a.cpp - the maze is the same on every launch.
# Drawn in the Task 2B format (see task2b/maze_2b.py): line 2 * row + 1 is
# a row of cells and the four chars from 4 * col are one cell. '---' is a
# wall on the +y / -y side of a cell (the lines above / below it), '|' a
# wall on its -x / +x side (the chars left / right of it). ' E ' in the
# bottom line marks the exit, on the -y side of (14, 14).
LAYOUT = (
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    "|                           |       |               |       |",
    "+---+---+   +---+   +   +   +   +   +---+   +---+   +---+   +",
    "|           |           |   |   |       |       |           |",
    "+   +   +   +   +---+---+---+   +---+   +   +   +---+---+   +",
    "|   |           |               |   |   |   |   |   |       |",
    "+   +---+---+---+   +---+   +---+   +   +---+   +   +   +---+",
    "|   |               |               |       |   |           |",
    "+   +   +---+---+---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |           |       |   |       |       |       |",
    "+   +---+   +   +   +---+---+   +   +   +---+---+   +---+   +",
    "|           |   |       |       |   |       |       |   |   |",
    "+   +   +   +   +---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |       |   |   |   |       |       |       |   |",
    "+   +   +   +---+---+   +   +   +   +   +   +---+   +   +   +",
    "|   |   |           |       |       |           |   |       |",
    "+   +   +---+---+   +   +---+---+   +---+---+   +   +---+---+",
    "|   |   |           |           |       |       |           |",
    "+   +   +   +---+---+---+---+   +   +   +   +---+---+---+   +",
    "|   |   |               |       |   |   |           |       |",
    "+   +---+---+   +---+   +   +---+   +   +---+   +   +   +---+",
    "|   |       |   |           |           |       |   |   |   |",
    "+   +   +   +   +   +---+   +---+---+   +   +---+   +   +   +",
    "|   |   |           |   |               |   |               |",
    "+   +   +---+---+---+   +---+---+---+   +   +---+   +---+   +",
    "|   |       |                   |       |       |       |   |",
    "+   +   +   +   +   +   +---+   +   +---+---+   +---+   +   +",
    "|   |   |       |   |           |   |       |   |       |   |",
    "+   +   +---+---+   +   +   +   +   +---+   +   +---+---+   +",
    "|               |           |               |               |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ E +",
)


def _wall_table():
    """{(row, col, d): True if side d of cell (row, col) is walled}."""
    walls = {}
    for row in range(N):
        for col in range(N):
            r, c = 2 * row + 1, 4 * col
            walls[row, col, 0] = LAYOUT[r][c + 4] == "|"                # +x: right
            walls[row, col, 1] = LAYOUT[r - 1][c + 1:c + 4] == "---"    # +y: above
            walls[row, col, 2] = LAYOUT[r][c] == "|"                    # -x: left
            walls[row, col, 3] = LAYOUT[r + 1][c + 1:c + 4] == "---"    # -y: below
    return walls


# WALLS[row, col, d] is True if there is a wall on side d of cell (row, col).
WALLS = _wall_table()


def cell_centre(cell):
    """World (x, y) of the centre of cell (row, col)."""
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


def neighbours(cell):
    """Open neighbouring cells of `cell`, as (direction, cell) pairs.
    The exit leads outside the maze and is not included."""
    row, col = cell
    out = []
    for d in range(4):
        n = (row + STEP[d][0], col + STEP[d][1])
        if not WALLS[row, col, d] and inside(*n):
            out.append((d, n))
    return out


# ----------------------------------------------------------------- robot
WHEEL_R = 0.017           # m, wheel radius
TRACK = 0.078             # m, wheel centre to wheel centre
TICKS_PER_REV = 360       # encoder ticks per wheel revolution
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0          # rad/s, the simulator's clamp
AXLE_BACK = 0.033         # m, the wheel axle is this far behind the chassis centre
NO_HIT = 2.0              # m, what a ToF reads when nothing is within its 2 m range

# ToF sensors: name -> (x, y) from the chassis centre, and the direction
# the ray points relative to the robot's heading. Note the names:
# "sl" / "sr" face FORWARD; "fl" / "fr" face the SIDES.
SENSORS = (
    ("sl", (-0.005, 0.034), 0.0),
    ("sr", (-0.005, -0.034), 0.0),
    ("fl", (0.036, 0.005), math.pi / 2),
    ("fr", (0.036, -0.005), -math.pi / 2),
)


def wrap(a):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wheels(v, omega):
    """(left, right) wheel speeds in rad/s for a forward speed v (m/s) and
    a turn rate omega (rad/s, positive = counter-clockwise / left)."""
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    return clamp(left, -WHEEL_MAX, WHEEL_MAX), clamp(right, -WHEEL_MAX, WHEEL_MAX)


# ------------------------------------------------------------- tuning
TURN_BASE = 1.2          # planner: any turn costs this many cells (stop + restart)
TURN_QUARTER = 0.8       # ... plus this per quarter turn of pivoting

V_MAX = 0.32             # m/s, cruise speed on straights while the position is confirmed
V_CAUTIOUS = 0.22        # m/s, cruise speed while any ToF disagrees with the map
DOUBT_ERR = 0.03         # m, a reading this far off the map's prediction = doubt
DOUBT_HOLD = 0.5         # s, stay cautious this long after the last doubtful reading
ACCEL = 0.6              # m/s^2, limit on how fast the speed command changes
DECEL = 0.4              # m/s^2, braking profile into a stop
K_END = 9.0              # 1/s, final approach: v = K_END * distance left (short creep)
STOP_TOL = 0.004         # m, "arrived" when this close and nearly still
K_HEAD = 8.0             # 1/s, heading P gain while driving
K_LAT = 15.0             # rad/m, steer back onto the centre line
W_MAX = 3.0              # rad/s, max pivot rate
ALPHA_DEC = 4.0          # rad/s^2, STARTING pivot braking (adapted per turn, see below) (gentle:
                         # the wheels slip ~18% when pivoting, so hard braking overshoots)
K_TURN = 10.0            # 1/s, final approach of a pivot (short exponential tail)
TURN_TOL = math.radians(1.5)
ALPHA_MIN, ALPHA_MAX = 2.5, 9.0   # rad/s^2, range the learned pivot braking stays in
OVERSHOOT_BAD = math.radians(2.0)  # a pivot overshooting more than this -> brake earlier
OVERSHOOT_OK = math.radians(0.5)   # a pivot overshooting less than this -> brake a bit later
SETTLE_RATE = 0.3        # rad/s, pivot done once the gyro reads below this

SLIP_INIT = 0.93         # ground travel per encoder distance (calibrated: 0.934; learned online)
SLIP_MIN, SLIP_MAX = 0.80, 1.05   # learned slip stays in this range
POS_GAIN = 0.08          # per frame, how hard a ToF reading pulls the pose
HEAD_GAIN = 0.05         # per frame, how hard the front pair pulls the heading
SIDE_RANGE = 0.25        # m, side ToF readings used for corrections
FRONT_RANGE = 1.0        # m, front ToF readings used for corrections
GATE = 0.03              # m, ignore a reading this far from the map's prediction
FRONT_GATE = 0.10        # m, wider gate when both front sensors see one flat wall
FRONT_SPAN = 0.068       # m, lateral spacing of the two forward sensors
EDGE_JUMP = 0.06         # m, a side reading changing this much in a frame = wall end
EDGE_WINDOW = 0.08       # m, how far a wall end may be from where we think we are
EDGE_GAIN = 0.8          # fraction of the edge error removed at once

STUCK_WINDOW = 0.6       # s of evidence before declaring the robot stuck
BACKOFF_SPEED = 0.06     # m/s, reversing away from a wall when stuck
BACKOFF_TIME = 0.6       # s
RELOC_SPAN = 0.10        # m, how far around the estimate to search when relocalizing
RELOC_PULL = 0.1        # fit-error units per metre moved: prefer staying put on ties
RECENTRE_TOL = 0.02       # m, after relocalizing, drive back to the cell centre if further off
RELOC_POOR = 0.008       # fit error above this after the first search -> search wider


def ray_length(px, py, ang):
    """What a ToF at world (px, py) pointing at `ang` should read in the known
    maze: walk the ray cell by cell and stop at the first walled side."""
    dx, dy = math.cos(ang), math.sin(ang)
    col = int(math.floor(px / PITCH + 0.5))
    row = N - 1 - int(math.floor(py / PITCH + 0.5))
    if not inside(row, col):
        return NO_HIT
    cx, cy = cell_centre((row, col))
    if dx > 1e-9:
        tx, sx, dtx = (cx + PITCH / 2 - px) / dx, 0, PITCH / dx
    elif dx < -1e-9:
        tx, sx, dtx = (cx - PITCH / 2 - px) / dx, 2, -PITCH / dx
    else:
        tx, sx, dtx = math.inf, 0, math.inf
    if dy > 1e-9:
        ty, sy, dty = (cy + PITCH / 2 - py) / dy, 1, PITCH / dy
    elif dy < -1e-9:
        ty, sy, dty = (cy - PITCH / 2 - py) / dy, 3, -PITCH / dy
    else:
        ty, sy, dty = math.inf, 1, math.inf
    while True:
        if tx < ty:
            t, side, comp = tx, sx, abs(dx)
        else:
            t, side, comp = ty, sy, abs(dy)
        if t > NO_HIT:
            return NO_HIT
        if WALLS[row, col, side]:
            return max(0.0, t - min(WALL_HALF / comp, 0.05))
        row, col = row + STEP[side][0], col + STEP[side][1]
        if not inside(row, col):
            return NO_HIT
        if tx < ty:
            tx += dtx
        else:
            ty += dty


# ============================================================== planning

def plan_route(start, heading, pellet):
    """Plan the route: from `start` facing `heading`, through `pellet`, out
    of the exit.

    start, pellet: (row, col) cells. heading: 0-3.
    Returns a list of actions for the solver to drive, for example
    [("run", 3), ("turn", 1), ("run", 2), ...] - "run n" drives n cells
    straight on, "turn d" pivots to face direction d. The last run should
    carry the robot out through the exit.

    Dijkstra over states (cell, heading, pellet collected?). Costs model
    TIME: a cell costs 1, a turn costs TURN_BASE (the stop and restart)
    plus TURN_QUARTER per quarter turn, so routes with fewer turns win. Passing through the pellet's cell sets "collected"; the
    goal is cell (14, 14) facing south with the pellet collected, followed
    by one more cell south, out through the exit.
    """
    start, pellet = tuple(start), tuple(pellet)
    s0 = (start, heading, start == pellet)
    goal = ((N - 1, N - 1), 3, True)
    dist = {s0: 0.0}
    prev = {}
    heap = [(0.0, 0, s0)]
    order = 1
    while heap:
        cost, _, s = heapq.heappop(heap)
        if cost > dist.get(s, math.inf):
            continue
        if s == goal:
            break
        cell, h, got = s
        moves = []
        for d, n in neighbours(cell):
            if d == h:
                moves.append(((n, h, got or n == pellet), 1.0, ("run", 1)))
        for d in range(4):
            if d != h:
                quarters = 2 if (d - h) % 4 == 2 else 1
                moves.append(((cell, d, got), TURN_BASE + TURN_QUARTER * quarters, ("turn", d)))
        for ns, c, act in moves:
            nc = cost + c
            if nc < dist.get(ns, math.inf):
                dist[ns] = nc
                prev[ns] = (s, act)
                heapq.heappush(heap, (nc, order, ns))
                order += 1
    if goal not in dist:
        return []

    steps = []
    s = goal
    while s != s0:
        s, act = prev[s]
        steps.append(act)
    steps.reverse()
    steps.append(("run", 1))  # out through the exit on the -y side of (14, 14)

    actions = []  # merge single steps into "run n" / one "turn d"
    for kind, val in steps:
        if actions and actions[-1][0] == kind:
            actions[-1] = (kind, actions[-1][1] + val) if kind == "run" else (kind, val)
        else:
            actions.append((kind, val))
    return actions


# ================================================================ solver

class Solver:
    """One sensor frame in, wheel speeds out."""

    def __init__(self):
        self.t = 0.0            # s of sim time since the first sensor frame
        self.spawn = None       # (dir, cell) of the robot at spawn
        self.pellet = None      # (row, col) cell of the pellet
        self.actions = None     # the plan, once made
        self.prev_ticks = None  # (left, right) encoder ticks last frame
        self.done = False
        # Pose estimate of the WHEEL AXLE (the point the robot pivots about),
        # in world metres / radians. Turns happen with the axle on a cell
        # centre, so the body sweeps a circle that stays inside the cell.
        self.x = self.y = self.th = 0.0
        self.k_slip = SLIP_INIT
        self.alpha = ALPHA_DEC  # learned pivot braking, rad/s^2
        self.doubt_until = 0.0  # drive at V_CAUTIOUS until this time
        self.overshoot = 0.0    # worst overshoot in the current pivot, rad
        self.gyro_bias = 0.0
        self.bias_n = 0
        self.v = 0.0            # last forward speed command, m/s
        self.v_meas = 0.0       # filtered odometry speed, m/s
        # Plan execution.
        self.i = 0              # index of the current action
        self.cell = None        # (row, col) the axle is on / heading for
        self.dir = 0            # heading of the current straight line, 0-3
        self.sub = None         # sub-state inside the current action
        self.sub_t = 0.0        # time spent in the current sub-state
        # Bookkeeping, as in the log line.
        self.wall_fixes = self.front_fixes = self.edge_fixes = 0
        self.prev_side = {}     # last side readings, for edge fixes
        self.enc_since = 0.0    # odometry travel since the last front/edge fix
        self.next_log = 10.0
        self.win_enc = self.win_corr = 0.0
        self.front_seen = False
        # Getting unstuck.
        self.got_pellet = False  # passed through the pellet's cell (estimated)
        self.last_r = {}         # sensor name -> (measured, map-predicted, angle)
        self.hist = []           # recent frames for stuck checks
        self.recover = None      # recovery state, or None
        self.recoveries = 0
        self.quiet_until = 0.0   # no stuck checks before this time
        self.action_t = 0.0      # time spent in the current action

    def log(self, text):
        print(f"[{self.t:7.2f}s] {text}", flush=True)

    def on_poses(self, msg):
        """pacbot/poses:
        {"bot": [row, col], "dir": 0-3}
        The chassis centre spawns on cell_centre(cell) (the axle is
        AXLE_BACK behind it)."""
        bot, d = msg.get("bot"), msg.get("dir")
        if isinstance(bot, list) and d is not None and self.actions is None:
            self.spawn = (d, tuple(bot))
            self.log(f"spawn {self.spawn[1]} facing {self.spawn[0]}")

    def on_pellets(self, msg):
        """pacbot/pellets:
        {"pellet": [row, col]} - the pellet's cell; it sits on the cell centre."""
        pellet = msg.get("pellet")
        if pellet and self.actions is None:
            self.pellet = tuple(pellet)
            self.log(f"pellet {self.pellet}")

    def step(self, s):
        """pacbot/sensors, one physics step:
        {"fl": m, "fr": m, "sl": m, "sr": m,      ToF distances (see SENSORS)
         "gyro": [x, y, z],                        rad/s; z is the turn rate
         "accel": [x, y, z],                       m/s^2
         "ticks_l": int, "ticks_r": int,           running encoder counts
         "dt": 0.01}                               s
        Returns (left, right) wheel speeds in rad/s."""
        self.t += s["dt"]
        ticks = (s["ticks_l"], s["ticks_r"])
        if self.prev_ticks is None:
            self.prev_ticks = ticks
        # Distance each wheel rolled since the last frame.
        d_left = (ticks[0] - self.prev_ticks[0]) * M_PER_TICK
        d_right = (ticks[1] - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = ticks
        turn_rate = s["gyro"][2]

        if self.done or self.spawn is None or self.pellet is None:
            return 0.0, 0.0
        if self.actions is None:
            self.actions = plan_route(self.spawn[1], self.spawn[0], self.pellet)
            self.log(f"plan: {self.actions}")
            d, cell = self.spawn
            cx, cy = cell_centre(cell)
            self.th = d * math.pi / 2
            self.x = cx - AXLE_BACK * math.cos(self.th)  # chassis centre is on
            self.y = cy - AXLE_BACK * math.sin(self.th)  # the cell centre
            self.cell, self.dir = cell, d
            if not self.actions:
                self.log("no route found - staying put")
                self.done = True
                return 0.0, 0.0

        # Hold still for the first 0.3 s and measure the gyro's bias.
        if self.bias_n < 30:
            self.gyro_bias += (turn_rate - self.gyro_bias) / (self.bias_n + 1)
            self.bias_n += 1
            return 0.0, 0.0

        rate = turn_rate - self.gyro_bias
        self.odometry(d_left, d_right, rate, s["dt"])
        driving = self.recover is None and (
            self.i < len(self.actions) and self.actions[self.i][0] == "run"
            or self.sub == "align" or self.sub == "exit")
        self.correct(s, driving)
        self.track_pellet()

        if self.t >= self.next_log:
            self.next_log += 10.0
            self.log(f"at ({self.x:.3f}, {self.y:.3f}) heading "
                     f"{math.degrees(self.th):.0f} deg, "
                     f"{len(self.actions) - self.i} action(s) left; slip "
                     f"{self.k_slip:.3f}, {self.wall_fixes} wall fixes, "
                     f"{self.front_fixes} front fixes, {self.edge_fixes} edge fixes, "
                     f"{self.recoveries} recoveries, pivot braking {self.alpha:.1f}")

        if self.recover is not None:
            return self.recovery(s, rate, s["dt"])
        cmd = self.control(rate, s["dt"])
        why = self.check_stuck(cmd, rate, s["dt"])
        if why:
            self.start_recovery(why)
            return 0.0, 0.0
        return cmd

    # ------------------------------------------------------------ estimation

    def odometry(self, d_left, d_right, rate, dt):
        """Dead reckoning: the gyro for heading (it does not care about wheel
        slip), the encoders scaled by the learned slip factor for distance."""
        ds = self.k_slip * (d_left + d_right) / 2
        dth = rate * dt
        mid = self.th + dth / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = wrap(self.th + dth)
        self.v_meas += (ds / dt - self.v_meas) * 0.2
        self._ds = ds
        self.enc_since += abs(ds)

    def correct(self, s, driving):
        """Compare each ToF with what the map says it should read from the
        current pose estimate, and nudge the pose to shrink the difference."""
        c, sn = math.cos(self.th), math.sin(self.th)
        r, pos = {}, {}
        for name, (ox, oy), rel in SENSORS:
            ax = ox + AXLE_BACK                      # sensor position from the axle
            px = self.x + ax * c - oy * sn
            py = self.y + ax * sn + oy * c
            ang = self.th + rel
            r[name] = (float(s[name]), ray_length(px, py, ang), ang)
            pos[name] = (px, py)

        # Sides ("fl" / "fr"): an edge fix when a reading jumps (the sensor
        # just passed the end of a side wall), then pull the pose across the
        # corridor.
        for name in ("fl", "fr"):
            m, e, ang = r[name]
            prev = self.prev_side.get(name)
            self.prev_side[name] = (m, pos[name])
            if driving and prev is not None:
                self.edge_fix(name, prev, m, pos[name], ang)
            if m < SIDE_RANGE and e < SIDE_RANGE and abs(m - e) < GATE:
                k = -POS_GAIN * (m - e)
                self.x += k * math.cos(ang)
                self.y += k * math.sin(ang)
                self.wall_fixes += 1

        # Corridor cross-check: if BOTH sides disagree with the map by the same
        # shift in opposite directions (one wall nearer, the other farther),
        # the corridor width confirms it - a real sideways error, so correct
        # it even when it is bigger than the normal gate.
        ml, el, angl = r["fl"]
        mr, er, _ = r["fr"]
        if max(ml, el, mr, er) < SIDE_RANGE:
            il, ir = ml - el, mr - er
            if abs(il) >= GATE and abs(il + ir) < 0.02 and abs(il - ir) / 2 < 0.09:
                shift = -POS_GAIN * (il - ir) / 2       # along the left-facing ray
                self.x += shift * math.cos(angl)
                self.y += shift * math.sin(angl)
                self.wall_fixes += 1
        self.last_r = r
        # Any nearby reading that disagrees with the map: slow down for a bit.
        if any(min(m, e) < 0.4 and abs(m - e) > DOUBT_ERR for m, e, _ in r.values()):
            self.doubt_until = self.t + DOUBT_HOLD

        # Front pair ("sl" / "sr"): heading from their difference, distance
        # along the corridor from their mean.
        msl, esl, _ = r["sl"]
        msr, esr, _ = r["sr"]
        flat = abs((msl - msr) - (esl - esr)) < 0.02 and abs(esl - esr) < 0.03
        gate = FRONT_GATE if flat else GATE
        ok = [m < FRONT_RANGE and e < FRONT_RANGE and abs(m - e) < gate
              for m, e in ((msl, esl), (msr, esr))]
        self.front_seen = False
        if ok[0] and ok[1] and flat and msl < 0.3 and msr < 0.3:
            dalpha = ((msl - msr) - (esl - esr)) / FRONT_SPAN
            if abs(dalpha) < 0.15:
                self.th = wrap(self.th + HEAD_GAIN * dalpha)
        if ok[0] or ok[1]:
            inns = [m - e for (m, e), good in zip(((msl, esl), (msr, esr)), ok) if good]
            inn = sum(inns) / len(inns)
            gain = POS_GAIN if len(inns) == 2 else POS_GAIN / 2
            k = -gain * inn
            self.x += k * c
            self.y += k * sn
            self.front_fixes += 1
            self.front_seen = True
            self.enc_since = 0.0
            if driving:
                # Learn the slip factor: over a stretch where the front wall
                # was in view, compare encoder travel with corrected travel.
                sign = 1.0 if self._ds >= 0 else -1.0
                self.win_enc += abs(self._ds)
                self.win_corr += k * sign
                if self.win_enc > 0.15:
                    ratio = (self.win_enc + self.win_corr) / self.win_enc
                    self.k_slip = clamp(self.k_slip * (1 + 0.3 * (ratio - 1)), SLIP_MIN, SLIP_MAX)
                    self.win_enc = self.win_corr = 0.0

    def edge_fix(self, name, prev, m, p, ang):
        """A side reading jumped between two frames: the sensor ray just
        crossed the end of a side wall. Wall ends sit on cell boundaries
        (plus/minus half a wall thickness), so find the boundary near the
        sensor where the map has the same kind of jump, and snap the pose
        along the corridor to it."""
        m_prev, p_prev = prev
        if abs(m - m_prev) < EDGE_JUMP or min(m, m_prev) > SIDE_RANGE:
            return
        d = round(self.th / (math.pi / 2)) % 4
        if abs(wrap(self.th - d * math.pi / 2)) > math.radians(6) or abs(self.v_meas) < 0.03:
            return
        ux, uy = math.cos(d * math.pi / 2), math.sin(d * math.pi / 2)
        fwd = 1.0 if self.v_meas > 0 else -1.0       # direction of motion along u
        opening = m > m_prev                          # wall -> opening
        s_now = p[0] * ux + p[1] * uy                 # sensor position along u
        s_mid = (s_now + p_prev[0] * ux + p_prev[1] * uy) / 2
        best = None
        k0 = math.floor(s_mid / PITCH - 0.5)
        for k in (k0 - 1, k0, k0 + 1, k0 + 2):
            b = (k + 0.5) * PITCH                     # a cell boundary along u
            # Wall material reaches WALL_HALF past the boundary.
            edge = b + (WALL_HALF if opening else -WALL_HALF) * fwd
            if abs(edge - s_mid) > EDGE_WINDOW:
                continue
            # Does the map flip the same way across this edge?
            before = ray_length(p[0] + (edge - s_now - 0.015 * fwd) * ux,
                                p[1] + (edge - s_now - 0.015 * fwd) * uy, ang)
            after = ray_length(p[0] + (edge - s_now + 0.015 * fwd) * ux,
                               p[1] + (edge - s_now + 0.015 * fwd) * uy, ang)
            if opening and not (before < SIDE_RANGE and after > before + EDGE_JUMP):
                continue
            if not opening and not (after < SIDE_RANGE and before > after + EDGE_JUMP):
                continue
            if best is None or abs(edge - s_mid) < abs(best - s_mid):
                best = edge
        if best is None:
            return
        err = best - s_mid
        shift = EDGE_GAIN * err
        self.x += shift * ux
        self.y += shift * uy
        self.edge_fixes += 1
        # The error built up over the travel since the last fix says how far
        # off the slip factor is.
        if 0.1 < self.enc_since < 1.5:
            rel = err * fwd / self.enc_since
            self.k_slip = clamp(self.k_slip * (1 + 0.5 * clamp(rel, -0.2, 0.2)), SLIP_MIN, SLIP_MAX)
        self.enc_since = 0.0

    # -------------------------------------------------------- getting unstuck

    def track_pellet(self):
        """Remember once the chassis has (by our estimate) passed the pellet."""
        if not self.got_pellet:
            cx = self.x + AXLE_BACK * math.cos(self.th)
            cy = self.y + AXLE_BACK * math.sin(self.th)
            px, py = cell_centre(self.pellet)
            if math.hypot(cx - px, cy - py) < 0.06:
                self.got_pellet = True
                self.log("pellet collected (by estimate)")

    def check_stuck(self, cmd, rate, dt):
        """Look at the last STUCK_WINDOW seconds; return a reason string if
        the robot is clearly not doing what it is being told to."""
        v_cmd = WHEEL_R * (cmd[0] + cmd[1]) / 2
        w_cmd = WHEEL_R * (cmd[1] - cmd[0]) / TRACK
        r = self.last_r
        self.hist.append((v_cmd, w_cmd, rate * dt, self.v_meas,
                          {k: r[k][0] for k in r}, {k: r[k][1] for k in r}))
        n = int(STUCK_WINDOW / dt)
        if len(self.hist) > n:
            del self.hist[0]
        if self.t < self.quiet_until or len(self.hist) < n:
            return None
        if self.sub == "exit":
            return None
        kind, val = self.actions[self.i] if self.i < len(self.actions) else ("", 0)
        limit = (4.0 + 2.5 * val) if kind == "run" else 8.0
        if self.action_t > limit:
            return f"action {self.actions[self.i]} took over {limit:.0f} s"
        h = self.hist
        # Pivoting hard, but the gyro says we are not turning AT ALL (checked
        # per frame: a pivot reversing out of an overshoot swings both ways,
        # so its total can be ~0 while it is clearly moving).
        if all(abs(f[1]) > 0.8 and abs(f[0]) < 0.02 for f in h) and max(abs(f[2]) for f in h) < 0.15 * dt:
            return "pivot not turning"
        # Driving, but the wheels have stopped.
        if all(abs(f[0]) > 0.05 for f in h) and all(abs(f[3]) < 0.02 for f in h):
            return "wheels stalled"
        # Odometry says a forward reading should have closed in, but it hasn't.
        if all(abs(f[0]) > 0.03 for f in h):
            for k in ("sl", "sr"):
                dm = abs(h[-1][4][k] - h[0][4][k])
                de = abs(h[-1][5][k] - h[0][5][k])
                if max(h[-1][4][k], h[0][4][k]) < 1.0 and de > 0.03 and dm < 0.004:
                    return f"{k} reading frozen while moving"
        # The map disagrees with two or more sensors the whole time.
        if all(sum(1 for k in f[4] if min(f[4][k], f[5][k]) < 0.4 and abs(f[4][k] - f[5][k]) > 0.04) >= 2
               for f in h):
            return "position estimate disagrees with the walls"
        return None

    def start_recovery(self, why):
        self.recoveries += 1
        self.log(f"stuck? {why} - recovering (#{self.recoveries})")
        # Back away from whatever we hit: against the last direction of travel,
        # or backwards if we were pivoting / standing.
        back = -1.0 if self.v >= -0.01 else 1.0
        if "disagrees" in why:
            back = 0.0                      # not touching anything: just stop
        self.recover = {"stage": "back", "t": 0.0, "dir": back, "flipped": False}
        self.v = 0.0
        self.hist.clear()

    def recovery(self, s, rate, dt):
        rc = self.recover
        rc["t"] += dt
        if rc["stage"] == "back":
            if rc["dir"] == 0.0 or rc["t"] > BACKOFF_TIME:
                rc["stage"], rc["t"] = "square", 0.0
                return 0.0, 0.0
            # Backing off doesn't move us either: try the other way once.
            if rc["t"] > 0.3 and abs(self.v_meas) < 0.01 and not rc["flipped"]:
                rc["dir"], rc["flipped"], rc["t"] = -rc["dir"], True, 0.0
            return wheels(rc["dir"] * BACKOFF_SPEED, 0.0)
        if rc["stage"] == "square":
            # Pivot to the nearest axis: the walls are then square-on to the
            # sensors, which makes relocalizing far more reliable.
            d = int(round(self.th / (math.pi / 2))) % 4
            err = wrap(d * math.pi / 2 - self.th)
            if (abs(err) < TURN_TOL and abs(rate) < SETTLE_RATE) or rc["t"] > 3.0:
                rc["stage"], rc["t"] = "settle", 0.0
                return 0.0, 0.0
            w = math.copysign(min(W_MAX, math.sqrt(2 * ALPHA_DEC * abs(err)), K_TURN * abs(err)), err)
            return wheels(0.0, w)
        if rc["stage"] == "settle":
            if rc["t"] < 0.3:
                return 0.0, 0.0
            self.relocalize(s)
            # Knowing where we are isn't enough if the robot itself is off its
            # lane: first drive it back to the centre of its cell.
            col = int(clamp(round(self.x / PITCH), 0, N - 1))
            row = int(clamp(N - 1 - round(self.y / PITCH), 0, N - 1))
            cx, cy = cell_centre((row, col))
            if math.hypot(cx - self.x, cy - self.y) > RECENTRE_TOL:
                rc.update(stage="face", t=0.0, goal=(cx, cy))
                return 0.0, 0.0
            return self.finish_recovery()
        if rc["stage"] in ("face", "go"):
            gx, gy = rc["goal"]
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            # Drive forwards or backwards, whichever needs the smaller turn.
            sign = 1.0 if abs(wrap(bearing - self.th)) <= math.pi / 2 else -1.0
            face = bearing if sign > 0 else wrap(bearing + math.pi)
            err = wrap(face - self.th)
            if rc["stage"] == "face":
                if (abs(err) < math.radians(3) and abs(rate) < SETTLE_RATE) or rc["t"] > 3.0 \
                        or dist < RECENTRE_TOL:
                    rc.update(stage="go", t=0.0)
                    return 0.0, 0.0
                w = math.copysign(min(W_MAX, math.sqrt(2 * ALPHA_DEC * abs(err)), K_TURN * abs(err)), err)
                return wheels(0.0, w)
            if dist < 0.005 or rc["t"] > 3.0:
                return self.finish_recovery()
            along = dist * math.cos(wrap(bearing - self.th)) * 1.0   # signed distance along heading
            v = clamp(3.0 * along, -0.10, 0.10)
            omega = clamp(K_HEAD * err, -2.0, 2.0) if dist > 0.02 else 0.0
            return wheels(v, omega)
        return 0.0, 0.0

    def finish_recovery(self):
        self.replan()          # starts with a turn that squares the robot up
        self.recover = None
        self.quiet_until = self.t + 1.0
        return 0.0, 0.0

    def pose_score(self, s, x, y, th):
        """How badly the four ToF readings fit a candidate axle pose."""
        c, sn = math.cos(th), math.sin(th)
        err = 0.0
        for name, (ox, oy), rel in SENSORS:
            ax = ox + AXLE_BACK
            e = ray_length(x + ax * c - oy * sn, y + ax * sn + oy * c, th + rel)
            m = float(s[name])
            if m < 1.5 or e < 1.5:
                # Long readings swing a lot with tiny heading errors: count
                # them less than short ones.
                err += min(abs(m - e), 0.15) / (1.0 + 3.0 * min(m, e))
        return err

    def relocalize(self, s):
        """Search nearby poses for the one that best explains the readings
        (heading comes from the gyro, so only a few degrees either way)."""
        x0, y0, th0 = self.x, self.y, self.th

        def cost(x, y, th):
            # Small penalty for moving away from the estimate, so a corridor
            # that fits equally well everywhere along it doesn't make us jump.
            return (self.pose_score(s, x, y, th)
                    + RELOC_PULL * math.hypot(x - x0, y - y0) + 0.05 * abs(th - th0))

        def search(centre, span, step, angs, keep):
            """Grid search around centre; returns the `keep` best candidates."""
            _, bx, by, bth = centre
            k = int(round(span / step))
            out = [centre]
            for da in angs:
                th = bth + math.radians(da)
                for i in range(-k, k + 1):
                    for j in range(-k, k + 1):
                        x, y = bx + i * step, by + j * step
                        out.append((cost(x, y, th), x, y, th))
            out.sort(key=lambda c: c[0])
            kept = []                     # best few, at least 3 cm apart
            for c in out:
                if all(math.hypot(c[1] - q[1], c[2] - q[2]) > 0.03 for q in kept):
                    kept.append(c)
                    if len(kept) == keep:
                        break
            return kept

        start = (cost(x0, y0, th0), x0, y0, th0)
        cands = search(start, RELOC_SPAN, 0.02, (-4, -3, -2, -1, 0, 1, 2, 3, 4), 8)
        if self.pose_score(s, *cands[0][1:]) > RELOC_POOR:
            # Still a poor fit: we are further off than that - look wider.
            cands = search(start, 2 * RELOC_SPAN, 0.02, (-6, -4.5, -3, -1.5, 0, 1.5, 3, 4.5, 6), 8)
        # The coarse grid can step past the true pose; refine several.
        best = min((search(c, 0.02, 0.005, (-0.6, 0, 0.6), 1)[0] for c in cands), key=lambda c: c[0])
        _, self.x, self.y, self.th = best
        self.log(f"relocalized by ({self.x - x0:+.3f}, {self.y - y0:+.3f}) m, "
                 f"fit error {self.pose_score(s, self.x, self.y, self.th):.3f}")

    def replan(self):
        """Plan afresh from the nearest cell and heading."""
        col = int(clamp(round(self.x / PITCH), 0, N - 1))
        row = int(clamp(N - 1 - round(self.y / PITCH), 0, N - 1))
        d = int(round(self.th / (math.pi / 2))) % 4
        target = (row, col) if self.got_pellet else self.pellet
        acts = plan_route((row, col), d, target)
        # Start with a "turn" to the current heading: it re-centres the axle
        # in the cell and squares the robot up before anything else.
        self.actions = [("turn", d)] + acts
        self.i, self.cell, self.dir = 0, (row, col), d
        self.sub, self.sub_t, self.action_t = None, 0.0, 0.0
        self.log(f"replanned from {(row, col)} facing {d}: {len(self.actions)} actions")

    # --------------------------------------------------------------- control

    def control(self, rate, dt):
        """Drive the current action; returns (left, right) wheel speeds."""
        self.sub_t += dt
        self.action_t += dt
        if self.sub == "exit":                      # past the exit: roll on out
            return self.drive_line(self.exit_target, self.dir, stop=False)[0]
        if self.i >= len(self.actions):
            return 0.0, 0.0
        kind, val = self.actions[self.i]
        last = self.i == len(self.actions) - 1

        if kind == "run":
            dr, dc = STEP[self.dir]
            target = (self.cell[0] + dr * val, self.cell[1] + dc * val)
            if last:
                # Final run leaves through the exit: don't stop, keep going.
                tx, ty = cell_centre(target)
                cmd, rem = self.drive_line((tx, ty), self.dir, stop=False)
                if rem < 0.0:
                    ux, uy = math.cos(self.dir * math.pi / 2), math.sin(self.dir * math.pi / 2)
                    self.sub, self.exit_target = "exit", (tx + ux, ty + uy)
                    self.log("through the exit")
                return cmd
            cmd, arrived = self.drive_line(cell_centre(target), self.dir, stop=True)
            if arrived:
                self.cell = target
                self.next_action()
            return cmd

        # kind == "turn": first put the axle on the cell centre, then pivot.
        if self.sub is None:
            self.sub, self.sub_t = "align", 0.0
        if self.sub == "align":
            cmd, arrived = self.drive_line(cell_centre(self.cell), self.dir, stop=True)
            if arrived or self.sub_t > 1.5:
                self.sub, self.sub_t = "pivot", 0.0
                self.v = 0.0
            return cmd
        err = wrap(val * math.pi / 2 - self.th)
        if self.sub_t <= dt:                      # first pivot frame
            self.turn_sign = 1.0 if err >= 0 else -1.0
            self.overshoot = 0.0
        self.overshoot = max(self.overshoot, -err * self.turn_sign)
        w = math.copysign(min(W_MAX, math.sqrt(2 * self.alpha * abs(err)), K_TURN * abs(err)), err)
        if abs(err) < TURN_TOL and abs(rate) < SETTLE_RATE:
            # Learn the pivot braking: overshot -> brake earlier next time;
            # clean -> brake slightly later. Never outside ALPHA_MIN..MAX.
            if self.overshoot > OVERSHOOT_BAD:
                self.alpha = max(ALPHA_MIN, self.alpha * 0.8)
            elif self.overshoot < OVERSHOOT_OK:
                self.alpha = min(ALPHA_MAX, self.alpha * 1.08)
            self.dir = val
            self.next_action()
            return 0.0, 0.0
        return wheels(0.0, w)

    def next_action(self):
        self.i += 1
        self.sub, self.sub_t, self.action_t = None, 0.0, 0.0

    def drive_line(self, target, d, stop):
        """Follow the centre line through `target` in direction d.
        Returns ((left, right), arrived) when stop is True, and
        ((left, right), distance left) when it is False."""
        ux, uy = math.cos(d * math.pi / 2), math.sin(d * math.pi / 2)
        ex, ey = target[0] - self.x, target[1] - self.y
        rem = ex * ux + ey * uy            # distance left along the line
        vmax = V_CAUTIOUS if self.t < self.doubt_until else V_MAX
        lat = -(ex * -uy + ey * ux)        # how far left of the line we are
        if stop:
            vt = math.copysign(min(vmax, math.sqrt(2 * DECEL * abs(rem)), K_END * abs(rem)), rem)
        else:
            vt = vmax
        step = ACCEL * 0.01
        self.v += clamp(vt - self.v, -step, step)
        th_ref = d * math.pi / 2
        if self.v > 0.02:
            th_ref -= clamp(math.atan(K_LAT * lat), -0.5, 0.5)
        omega = clamp(K_HEAD * wrap(th_ref - self.th), -W_MAX, W_MAX)
        cmd = wheels(self.v, omega)
        if not stop:
            return cmd, rem
        arrived = abs(rem) < STOP_TOL and abs(self.v_meas) < 0.02
        if arrived:
            self.v = 0.0
        return cmd, arrived


# ================================================================== main

def _client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:  # paho-mqtt < 2
        return mqtt.Client()


def main():
    solver = Solver()
    client = _client()

    def send(cli, left, right):
        # The simulator parses exactly this shape - keep json's defaults.
        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))

    def on_message(cli, _userdata, msg):
        if msg.topic == TOPIC_SENSORS:
            send(cli, *solver.step(json.loads(msg.payload)))
        elif msg.topic == TOPIC_POSES:
            if msg.payload:  # an empty retained message just clears the topic
                solver.on_poses(json.loads(msg.payload))
        elif msg.topic == TOPIC_PELLETS:
            if msg.payload:
                solver.on_pellets(json.loads(msg.payload))
        elif msg.topic == TOPIC_RESULT:
            solver.log(f"result: {msg.payload.decode()}")
            solver.done = True
            send(cli, 0.0, 0.0)
            cli.disconnect()

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    for topic in (TOPIC_POSES, TOPIC_PELLETS, TOPIC_RESULT, TOPIC_SENSORS):
        client.subscribe(topic)
    print("connected, waiting for the simulator", flush=True)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
