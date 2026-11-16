"""PB Task 2B - ARC variant: corners are driven as smooth quarter-circle arcs
without stopping, except at pellet cells, near the ghost and for U-turns,
where it still stops and pivots. Otherwise the same as task_2b_fast.py.
Rename to task_2b_boilerplate.py to submit.

Boilerplate for PB Task 2B.

All the plumbing, none of the thinking. It connects to the broker, loads the
official maze, receives the pellet positions for this run, and publishes a
wheel velocity command on every sensor frame. Your job is the one TODO block
in `decide()`.

Run it in two terminals, from the folder you unpacked:

    ./task_2b_launch                 # terminal 1: the simulator
    python3 task_2b_boilerplate.py   # terminal 2: this file

Neither takes a maze argument. The official maze is built into both, so there
is no path to get wrong and no way for your planner and the simulator to end
up looking at different walls. If the broker is not already running:

    sudo systemctl enable --now mosquitto

Everything you need is in this file: the maze, where you start and finish, and
every message the simulator will send you. Read on.

------------------------------------------------------------------------------
WHAT YOU ARE GIVEN

The maze. Fixed for the whole Sub-Task and published, so you may read it,
precompute against it, and plan routes offline. `Maze.official()` hands it to
you already parsed, from a copy embedded in maze_2b.py, which is the same text
built into the simulator. This is deliberately
not a mapping task - the ToF sensors are for driving accurately, not for
discovering walls.

The pellets. THREE cells, delivered over `pacbot/pellets` when the run starts,
and different on every single run. You cannot hardcode a route. Every possible
pellet set has been checked to need the same optimal driving time, so no draw
is luckier than another; the only thing that separates two teams is how well
they plan and drive.

------------------------------------------------------------------------------
THE MAZE

This is the whole thing. It never changes, so plan against it offline.
`Maze.official()` returns it already parsed; the drawing below is the same
maze for reading with your eyes.

        col    0   1   2   3   4   5   6   7   8   9  10  11  12  13  14
             +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
     row 0   |                           |       |       |               |
             +   +---+---+   +---+   +   +   +   +   +   +   +---+   +---+
     row 1   |   |               |   |       |       |   |       |       |
             +   +   +   +---+   +   +   +   +---+   +   +   +---+---+   +
     row 2   |   |   |       |   |       |       |   |               |   |
             +   +   +   +---+   +   +   +---+   +   +---+   +   +   +   +
     row 3   |       |               |           |           |   |       |
             +---+---+   +---+---+   +   +---+   +---+---+---+   +   +---+
     row 4   |   |               |   |               |                   |
             +   +   +---+   +   +   +---+---+---+   +---+   +   +---+   +
     row 5   |       |       |   |               |           |           |
             +   +---+   +---+   +   +---+---+   +   +---+---+   +---+   +
     row 6   |       |   |       |           |       |           |       |
             +---+   +   +   +---+---+---+   +---+---+   +   +---+   +---+
     row 7   |                           |               |               E
             +   +---+---+   +---+---+   +   +   +---+---+   +   +---+   +
     row 8   |           |                   |               |           |
             +   +   +   +   +---+---+---+   +---+   +   +---+   +---+---+
     row 9   |   |   |       |                   |   |                   |
             +   +   +---+   +   +---+   +---+   +   +---+---+   +---+   +
     row 10  |           |           |           |           |   |       |
             +   +---+   +---+---+   +   +---+---+   +   +   +   +   +   +
     row 11  |       |           |                   |   |   |   |   |   |
             +---+   +   +---+   +---+---+   +---+   +   +   +   +   +   +
     row 12  |       |               |           |       |           |   |
             +   +   +---+   +---+   +   +---+   +---+   +   +---+   +   +
     row 13  |   |           |       |       |               |           |
             +   +   +   +---+   +   +---+   +   +   +---+   +   +---+   +
     row 14  S       |           |                       |               |
             +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

You START outside the west border at the `S` gap on row 14, facing EAST, and
drive into cell (14, 0). You FINISH by leaving cell (7, 14) through the `E`
gap in the east border. Crossing that plane is the only thing that ends a run
successfully.

Coordinates. Cells are (row, col), both zero-based, with row 0 at the TOP of
the drawing and col 0 at the LEFT. Headings are

    EAST = 0    NORTH = 1    WEST = 2    SOUTH = 3

The order runs counter-clockwise, so (heading + 1) % 4 is a 90 degree turn to
the LEFT, and (heading + 3) % 4 is a turn to the RIGHT. You spawn facing EAST,
which is heading 0.

Cell centres in metres. This is the frame the ghost's x/y arrive in, and the
one to dead-reckon in:

    x = col * 0.22
    y = (14 - row) * 0.22

So cell (14, 0) sits at (0.00, 0.00) and cell (0, 14) at (3.08, 3.08). Driving
EAST increases x; driving NORTH increases y. Going back the other way:

    col = round(x / 0.22)
    row = 14 - round(y / 0.22)

You spawn at (-0.15, 0.00), in the short chute outside the west border, so
your first move is eastward through the gap. The run ends when your chassis
crosses x = 3.19, which is the east border plane half a pitch beyond the
centres of column 14.

Asking the maze about walls, rather than working it out from the drawing:

    maze.can_move((7, 0), Maze.EAST)     # -> True / False
    maze.grid                            # 15
    maze.pitch                           # 0.22
    maze.spawn_row, maze.exit_row        # 14, 7

------------------------------------------------------------------------------
THE WIRE PROTOCOL

  pacbot/sensors    simulator -> you, 100 times a second
    {"fl": .., "fr": .., "sl": .., "sr": ..,
     "gyro":  [wx, wy, wz], "accel": [ax, ay, az],
     "ticks_l": .., "ticks_r": .., "dt": .., "t": ..}

    The physics runs at 500 Hz; you see every fifth step, so dt is always
    0.010. The simulator does not wait for you between frames - it keeps
    stepping and the wheels hold your last command.

    fl  metres to the wall on the LEFT,  90 deg left of forward  (+y)
    fr  metres to the wall on the RIGHT, 90 deg right of forward (-y)
    sl  metres to the wall AHEAD, 0 deg, from left of centre     (+x)
    sr  metres to the wall AHEAD, 0 deg, from right of centre    (+x)

      All four rays are axis-aligned, so a reading IS the perpendicular
      distance. No trigonometry needed.

      Do not be misled by the names: `fl`/`fr` look sideways and `sl`/`sr`
      look forward. They are inherited from Task 1B so your 1B code still
      works.

      RANGE. The sensors see 2.0 m and no further, like the real hardware.
      A reading of 2.0 means "at least 2 m away, or nothing there at all" -
      you cannot tell those apart, and neither can a real ToF module. Every
      reading is therefore in [0.0, 2.0].

      THE PELLETS ARE INVISIBLE TO THESE SENSORS. They are simulated as
      sites, which the rangefinders cannot see, so you cannot sweep the maze
      looking for them - a ToF ray passes straight through a pellet and
      reports the wall behind it. The only source of pellet positions is the
      pacbot/pellets topic below.

    gyro[2]        yaw rate about z, rad/s. Integrate it to measure turns.
    accel          body-frame acceleration, m/s^2.
    ticks_l,       RUNNING encoder counts, 360 per wheel revolution, positive
    ticks_r        forward. These are cumulative totals, not rates: subtract
                   successive readings to get how far each wheel turned, so a
                   dropped message loses nothing.

                       dl = ticks_l - prev_ticks_l
                       dr = ticks_r - prev_ticks_r
                       distance = WHEEL_RADIUS * ((dl + dr) / 2) * 2*pi/360
    dt             seconds covered by this frame.
    t              simulator clock, seconds since the run started.
  pacbot/pellets    simulator -> you, at startup and after each collection
    {"pellets": [[row, col], [row, col], [row, col]]}

    The pellets STILL on the board. The list shrinks as you collect them, and
    an empty list means all three are yours. Retained, so a late subscriber
    still gets it.

    Wrapped in an object with a named key, like Task 2A's pellet message,
    so the topic can gain more fields later without breaking your parser.

  pacbot/ghost      simulator -> you, every sensor frame
    {"row": .., "col": .., "x": .., "y": ..}

    Where the patrolling ghost is RIGHT NOW: its cell, and its exact position
    in metres. RETAINED, so subscribing late still tells you where it is.

    It never chases you, but touching it ends your run with whatever pellets
    you have already collected. Its route is never published: watch it and
    work the circuit out yourself. It is slower than you are and it is on a
    circuit, so waiting always works.

  pacbot/result     simulator -> you, ONCE, when the run ends
    {"solved": true, "collisions": 0}

    Sent when the maze is solved, or when the window is closed on a run that
    was not solved. It tells you the run is over and roughly how it went.
    "solved" means all three pellets AND out through the exit; exiting with
    two pellets is not solved.

    It deliberately carries no score, no time and no pellet detail. Those are
    in the sealed result.json only. Not retained, so it can never be mistaken
    for the previous run's outcome.

  pacbot/wheel_vel  you -> simulator
    {"left": <rad/s>, "right": <rad/s>}

    Clamped to +-30 rad/s. Publish one per sensor frame.

    The simulator does NOT wait for you. It keeps stepping at the wall clock
    and the wheels hold whatever command arrived last, exactly as a real
    robot coasts on its last instruction between control updates. Miss a
    frame and you simply steer on a slightly older reading; nothing breaks.

    The keys must appear in THIS ORDER: the simulator matches the message
    against a fixed pattern, so {"right": .., "left": ..} is discarded
    without an error. Building the dict as {"left": .., "right": ..} and
    passing it to json.dumps gives the right shape, as the code below does.

------------------------------------------------------------------------------
GEOMETRY YOU WILL NEED

    cell pitch      0.22 m          maze.pitch
    wheel radius    0.017 m
    half track      0.039 m         wheel centre to robot centreline
    grid            15 x 15         maze.grid

A pellet is collected when the chassis centre passes within 0.06 m of the cell
centre, so clipping a corner does not count. The run ends when the chassis
crosses the exit plane on the east border.
"""
import bisect
import heapq
import json
import math

import paho.mqtt.client as mqtt

import maze_2b
from maze_2b import Maze

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_PELLETS = "pacbot/pellets"      # simulator publishes, this file subscribes
TOPIC_GHOST = "pacbot/ghost"          # simulator publishes, this file subscribes
TOPIC_RESULT = "pacbot/result"        # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes

WHEEL_RADIUS = 0.017  # m
HALF_TRACK = 0.039    # m, wheel centre to centreline
WHEEL_LIMIT = 30.0    # rad/s, the simulator clamps to this
TICKS_PER_REV = 360   # encoder counts per wheel revolution
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV   # radians of wheel turn per tick


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def wheels(forward, turn):
    """Differential drive mixer: +turn steers LEFT (counter-clockwise)."""
    return forward - turn, forward + turn



# =========================================================== maze geometry
# Everything below is built from the official maze (maze_2b), so the planner
# and the ToF predictions use exactly the walls the simulator uses.
MAZE = Maze.official()
N = MAZE.grid                  # 15 cells per side
PITCH = MAZE.pitch             # 0.22 m, cell centre to cell centre
WALL_HALF = 0.005              # m, half the wall thickness
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))   # (d_row, d_col) for E, N, W, S
SPAWN_CELL = (MAZE.spawn_row, 0)            # (14, 0): first cell inside the maze
EXIT_CELL = (MAZE.exit_row, N - 1)          # (7, 14): leave it heading EAST
EXIT_X = (N - 1) * PITCH + PITCH / 2        # 3.19 m: crossing this ends the run


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


def cell_centre(cell):
    """World (x, y) of the centre of cell (row, col)."""
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def _wall_table():
    """{(row, col, d): True if side d of cell (row, col) is walled}.
    The spawn and exit gaps count as open; leaving the grid anywhere else
    is impossible."""
    walls = {(r, c, d): not MAZE.can_move((r, c), d)
             for r in range(N) for c in range(N) for d in range(4)}
    # maze.can_move() says False at the two border gaps (leaving the grid is
    # not "ordinary movement"), but for predicting the ToF sensors they are
    # openings, not walls.
    walls[SPAWN_CELL[0], SPAWN_CELL[1], 2] = False
    walls[EXIT_CELL[0], EXIT_CELL[1], 0] = False
    return walls


WALLS = _wall_table()


def neighbours(cell):
    """Open neighbouring cells of `cell` inside the maze, as (direction, cell)."""
    row, col = cell
    out = []
    for d in range(4):
        n = (row + STEP[d][0], col + STEP[d][1])
        if not WALLS[row, col, d] and inside(*n):
            out.append((d, n))
    return out


# ----------------------------------------------------------------- robot
WHEEL_R = WHEEL_RADIUS    # 0.017 m
TRACK = 2 * HALF_TRACK    # 0.078 m, wheel centre to wheel centre
M_PER_TICK = WHEEL_R * TICK_RAD
WHEEL_MAX = WHEEL_LIMIT   # 30 rad/s
AXLE_BACK = 0.033         # m, the wheel axle is this far behind the chassis centre
NO_HIT = 2.0              # m, what a ToF reads when nothing is within range
SPAWN_XY = (-0.15, 0.0)   # m, chassis centre at spawn, facing EAST

# ToF sensors: name -> (x, y) from the chassis centre, and the direction the
# ray points relative to the heading (same robot as Task 2A).
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


def wheel_speeds(v, omega):
    """(left, right) wheel speeds in rad/s for a forward speed v (m/s) and
    a turn rate omega (rad/s, positive = counter-clockwise / left)."""
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    return clamp(left, -WHEEL_MAX, WHEEL_MAX), clamp(right, -WHEEL_MAX, WHEEL_MAX)


# ------------------------------------------------------------- tuning
# Planner costs, in SECONDS of driving (the clock is what is scored).
T_CELL = 0.78            # s per cell on a straight (cruise incl. a share of accel)
T_STOP = 1.0             # s extra for every stop + restart (each turn needs one)
T_QUARTER = 0.8          # s per quarter turn of pivoting
T_ARC_TURN = 0.4         # s for a corner driven as an arc (no stop) - planner cost
AGAINST_GHOST_COST = 20.0  # s extra per loop cell driven against the ghost's direction
TURN_IN_GHOST_COST = 4.0  # s extra per turn made on the ghost's loop
GHOST_AREA_COST = 6.0    # s penalty per cell entered on the ghost's loop: go round it
                         # unless a pellet makes that impossible or a big detour

# Time model for the ghost check (scaled online by the measured k_time).
T_RUN_EXTRA = 1.1        # s, accelerating + braking on top of cells / V_MAX
T_TURN_FIXED = 0.9       # s, settling at the end of a pivot
T_TURN_Q = 0.75          # s per quarter turn
T_ARC = 0.9              # s, one arc corner

# Ghost.
GHOST_SAFE = 0.18        # m, never let the predicted ghost come closer than this
                         # to the axle (robot reach ~0.10 + ghost ~0.04 + margin)
SAFE_CELL_DIST = 0.21    # m, a cell this far from the ghost's path is a safe stop
                         # (a neighbour of the loop is 0.22 away: the ghost cannot reach it)
SLACK_BASE = 0.6         # s, timing uncertainty at the start of a plan check
SLACK_FRAC = 0.10        # ... growing by this fraction of the predicted time
                         # (re-checked at every stop, so it never has to cover a long stretch)
GHOST_BIN = 0.05         # s, resolution of the learned lap
LEARN_TIMEOUT = 120.0    # s, give up learning the loop after this (never seen so far)
WAIT_STEP = 0.2          # s, resolution of the "when to go" search

V_MAX = 0.38             # m/s, cruise speed on straights while the position is confirmed
V_ARC = 0.24             # m/s, fastest arc speed ever used
V_ARC_MIN = 0.07         # m/s, below this arcs are not worth it: stop and pivot instead
ARC_SWING = 0.25         # arc speed = ARC_SWING * (measured turn acceleration) * ARC_R,
                         # which keeps the swing-out while the turn builds up near 1 cm
ARC_LEARN_PIVOTS = 2     # pivots measured before the first arc is allowed
ARC_BAD_R = 0.02         # m, an arc straying this far from its circle counts as bad
ARC_R = PITCH / 2        # m, arc radius: the axle swings round the corner post
K_ARC_H = 4.0            # 1/s, heading correction in an arc
K_ARC_R = 15.0           # rad/s per m, radius correction in an arc
ARC_GHOST_CLEAR = 0.66   # m, no arcs within this of the ghost's path (stay able to stop)
V_CAUTIOUS = 0.28        # m/s, cruise speed while any ToF disagrees with the map
DOUBT_ERR = 0.03         # m, a reading this far off the map's prediction = doubt
DOUBT_HOLD = 0.5         # s, stay cautious this long after the last doubtful reading
ACCEL = 1.0              # m/s^2, limit on how fast the speed command changes
DECEL = 0.75              # m/s^2, braking profile into a stop
K_END = 12.0              # 1/s, final approach: v = K_END * distance left (short creep)
STOP_TOL = 0.004         # m, "arrived" when this close and nearly still
K_HEAD = 6.0             # 1/s, heading P gain while driving
K_LAT = 8.0             # rad/m, steer back onto the centre line (at V_STEER_REF)
V_STEER_REF = 0.25       # m/s, above this speed the centring is softened
W_STEER = 1.2            # rad/s, max turn rate while driving straight
W_MAX = 3.0              # rad/s, max pivot rate
ALPHA_DEC = 4.0          # rad/s^2, braking profile into the end of a pivot (gentle:
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

def plan_route(start, heading, pellets, near_ghost=frozenset(), ghost_edges=frozenset(),
               forbid_ghost=False, no_arc=frozenset()):
    """Plan the fastest route: from `start` facing `heading`, through every
    cell in `pellets` (in whatever order is quickest), out of the exit.

    Dijkstra over states (cell, heading, which pellets are collected). Costs
    are seconds: T_CELL per cell, and a turn costs T_STOP (it needs a stop
    and a restart) plus T_QUARTER per quarter turn - so a route with fewer
    turns wins even if it is a little longer. Cells in `near_ghost` cost a
    extra, so routes go round the ghost's loop when they can. Inside the loop,
    moving the same way as the ghost (following it) is cheap, moving against
    it (meeting it head-on) and turning are expensive. With forbid_ghost the
    loop is a wall: pellets only reachable through it are skipped.
    Passing through a pellet's cell collects it; the goal is the exit cell
    facing EAST with everything collected, then one more cell east, out.

    Returns (actions, seconds): actions like [("run", 3), ("turn", 1), ...];
    "run n" drives n cells straight on, "turn d" pivots to face d.
    """
    start = tuple(start)
    pellets = [tuple(p) for p in pellets]
    if forbid_ghost:
        # Keep only the pellets reachable without touching the ghost's loop.
        seen, todo = {start}, [start]
        while todo:
            c = todo.pop()
            for _, n in neighbours(c):
                if n not in seen and n not in near_ghost:
                    seen.add(n)
                    todo.append(n)
        pellets = [p for p in pellets if p in seen]
    full = (1 << len(pellets)) - 1

    def mask_of(cell, m):
        for k, p in enumerate(pellets):
            if p == cell:
                m |= 1 << k
        return m

    s0 = (start, heading, mask_of(start, 0))
    goal = (EXIT_CELL, 0, full)
    dist = {s0: 0.0}
    prev = {}
    heap = [(0.0, 0, s0)]
    order = 1
    while heap:
        cost, _, st = heapq.heappop(heap)
        if cost > dist.get(st, math.inf):
            continue
        if st == goal:
            break
        cell, h, m = st
        moves = []
        for d, n in neighbours(cell):
            if d == h:
                c = T_CELL
                if n in near_ghost:
                    if forbid_ghost:
                        continue
                    c += GHOST_AREA_COST
                    if (n, cell) in ghost_edges and (cell, n) not in ghost_edges:
                        c += AGAINST_GHOST_COST          # head-on with the ghost
                moves.append(((n, h, mask_of(n, m)), c, ("run", 1)))
        for d in range(4):
            if d != h:
                quarters = 2 if (d - h) % 4 == 2 else 1
                if quarters == 1 and cell not in pellets and cell not in no_arc:
                    c = T_ARC_TURN                       # driven through as an arc
                else:
                    c = T_STOP + T_QUARTER * quarters    # stop and pivot
                if cell in near_ghost:
                    c += TURN_IN_GHOST_COST              # standing still on its path
                moves.append(((cell, d, m), c, ("turn", d)))
        for ns, c, act in moves:
            nc = cost + c
            if nc < dist.get(ns, math.inf):
                dist[ns] = nc
                prev[ns] = (st, act)
                heapq.heappush(heap, (nc, order, ns))
                order += 1
    if goal not in dist:
        return [], math.inf

    steps = []
    st = goal
    while st != s0:
        st, act = prev[st]
        steps.append(act)
    steps.reverse()
    steps.append(("run", 1))  # out through the E gap on the east side of (7, 14)

    actions = []  # merge single steps into "run n" / one "turn d"
    for kind, val in steps:
        if actions and actions[-1][0] == kind:
            actions[-1] = (kind, actions[-1][1] + val) if kind == "run" else (kind, val)
        else:
            actions.append((kind, val))
    return actions, dist[goal]


def make_arcs(actions, cell, d, pellets, no_arc):
    """Turn every quarter-turn that sits between two runs into an ("arc", d)
    - driven without stopping - unless it is at a pellet (the chassis must
    pass the cell centre to collect it) or close to the ghost (we must be able
    to stop and wait there). U-turns and the first move stay pivots."""
    out = list(actions)
    for k, (kind, val) in enumerate(out):
        if kind == "turn":
            q = (val - d) % 4
            if (q in (1, 3) and 0 < k < len(out) - 1 and out[k - 1][0] == "run"
                    and out[k + 1][0] == "run" and cell not in pellets and cell not in no_arc):
                out[k] = ("arc", val)
            d = val
        elif kind == "arc":
            d = val
        else:
            cell = (cell[0] + STEP[d][0] * val, cell[1] + STEP[d][1] * val)
    return out


def route_string(actions, heading):
    """The plan as F / L / R letters (one F per cell), as in the reference log."""
    out = []
    h = heading
    for kind, val in actions:
        if kind == "run":
            out.append("F" * val)
        else:
            q = (val - h) % 4
            letter = {1: "L", 2: "LL", 3: "R"}[q]
            out.append(letter.lower() if kind == "arc" else letter)
            h = val
    return "".join(out)


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
        self.last_r = {}         # sensor name -> (measured, map-predicted, angle)
        self.hist = []           # recent frames for stuck checks
        self.recover = None      # recovery state, or None
        self.recoveries = 0
        self.quiet_until = 0.0   # no stuck checks before this time
        self.action_t = 0.0      # time spent in the current action
        # Task 2B.
        self.pellets = None      # cells still on the board (from pacbot/pellets)
        self.expect = {}         # pellet -> index of the action that collects it
        self.g_samples = []      # ghost (t, x, y), one per frame
        self.g_trans = []        # ghost (t, cell) at every change of cell
        self.loop = None         # the ghost's cells, in patrol order, once learned
        self.g_period = None     # s, time for one lap
        self.g_cycle = None      # [(x, y)] one lap of ghost positions, BIN apart
        self.near_ghost = frozenset()   # cells within reach of the ghost's path
        self.no_arc = frozenset()       # cells where corners must stop and pivot
        self.arc_v = 0.0                # speed for the arc being driven
        self.alpha_meas = []            # turn acceleration measured in each pivot, rad/s^2
        self.arc_scale = 1.0            # shrinks if arcs track badly
        self.arcs_off = False           # True: every corner is a stop-and-pivot
        self.arc_err = 0.0              # worst radius error in the current arc
        self.n_arcs = 0
        self.next_watch_log = 0.0
        self.chunk_end = 0       # actions before this index are already cleared
        self.boundary_checked = -1
        self.wait_until = None   # holding still until this time (ghost passing)
        self.gated_i = -1        # action index the ghost check last ran for
        self.waited_once = False
        self.give_up_guarded = False   # skipping pellets that only the ghost's loop reaches
        self.k_time = 1.0        # learned: real action time / modelled time
        self.quiet = False

    def log(self, text):
        if not self.quiet:
            print(f"[{self.t:7.2f}s] {text}", flush=True)

    # ------------------------------------------------------------ 2B inputs

    def set_pellets(self, cells):
        """Latest pacbot/pellets list: the cells STILL on the board."""
        self.pellets = [tuple(c) for c in cells]

    def set_ghost(self, g):
        """Latest pacbot/ghost payload, sampled once per sensor frame."""
        if g is None:
            return
        x, y = float(g["x"]), float(g["y"])
        cell = (int(g["row"]), int(g["col"]))
        self.g_samples.append((self.t, x, y))
        if len(self.g_samples) > 30000:          # keep ~5 minutes
            del self.g_samples[:5000]
        if not self.g_trans or self.g_trans[-1][1] != cell:
            self.g_trans.append((self.t, cell))
            if self.loop is None:
                self.learn_loop()

    def step(self, s):
        """pacbot/sensors, one physics step:
        {"fl": m, "fr": m, "sl": m, "sr": m,      ToF distances (see SENSORS)
         "gyro": [x, y, z],                        rad/s; z is the turn rate
         "accel": [x, y, z],                       m/s^2
         "ticks_l": int, "ticks_r": int,           running encoder counts
         "dt": 0.01, "t": s}                       s
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

        if self.done or self.pellets is None:
            return 0.0, 0.0

        # Before the first move: stand still (the clock has not started, so
        # this is free), measure the gyro bias, and watch the ghost until its
        # whole loop is known.
        if self.actions is None:
            self.gyro_bias += (turn_rate - self.gyro_bias) / (self.bias_n + 1)
            self.bias_n += 1
            if self.loop is None and self.t < LEARN_TIMEOUT:
                if self.t >= self.next_watch_log:
                    self.next_watch_log += 5.0
                    self.log(f"watching the ghost: {len(self.g_trans)} cell changes seen")
                return 0.0, 0.0
            if self.loop is None:
                self.log("ghost loop not learned - planning without it (will keep watching)")
            self.first_plan()
            if not self.actions:
                self.log("no route found - staying put")
                self.done = True
                return 0.0, 0.0

        rate = turn_rate - self.gyro_bias
        self.odometry(d_left, d_right, rate, s["dt"])
        driving = self.recover is None and self.wait_until is None and (
            self.i < len(self.actions) and self.actions[self.i][0] == "run"
            or self.sub == "align" or self.sub == "exit")
        if self.i < len(self.actions) and self.actions[self.i][0] == "arc":
            driving = False
        self.correct(s, driving)

        if self.t >= self.next_log:
            self.next_log += 10.0
            self.log(f"at ({self.x:.3f}, {self.y:.3f}) heading "
                     f"{math.degrees(self.th):.0f} deg, "
                     f"{len(self.actions) - self.i} action(s) left, "
                     f"{3 - len(self.pellets)}/3 pellets; slip "
                     f"{self.k_slip:.3f}, {self.wall_fixes} wall fixes, "
                     f"{self.front_fixes} front fixes, {self.edge_fixes} edge fixes, "
                     f"{self.recoveries} recoveries, pivot braking {self.alpha:.1f}")

        if self.recover is not None:
            return self.recovery(s, rate, s["dt"])

        # Waiting for the ghost to pass: hold still.
        if self.wait_until is not None:
            if self.t < self.wait_until:
                return 0.0, 0.0
            self.wait_until = None
            self.waited_once = True
            self.hist.clear()
        # At an action boundary: missed pellets? Is the next stretch safe?
        if self.sub is None and self.i < len(self.actions) and self.sub != "exit":
            if self.i != self.boundary_checked:
                self.boundary_checked = self.i
                if self.check_missed():
                    return 0.0, 0.0
            if ((self.loop is not None or self.i >= self.chunk_end) and self.gated_i != self.i
                    and self.standing()):
                self.gated_i = self.i             # once per action, not every frame
                w = self.gate()
                if w > 0.05:
                    self.wait_until = self.t + w
                    self.boundary_checked = -1
                    self.gated_i = -1             # look again when the wait is over
                    return 0.0, 0.0
        cmd = self.control(rate, s["dt"])
        why = self.check_stuck(cmd, rate, s["dt"])
        if why:
            self.start_recovery(why)
            return 0.0, 0.0
        return cmd

    # ---------------------------------------------------------------- planning

    def standing(self):
        """Is the robot stopped at this action boundary? Only then may the
        ghost check make it wait (arcs and the runs either side keep moving)."""
        i, a = self.i, self.actions
        if i == 0:
            return True
        if a[i][0] == "arc" or a[i - 1][0] == "arc":
            return False
        return a[i - 1][0] == "turn" or a[i][0] == "turn"

    def first_plan(self):
        """Plan the whole run from the spawn chute, before moving."""
        if self.loop is not None:
            self.log(f"ghost loop: {len(self.loop)} cells, period {self.g_period:.1f} s, "
                     f"cells {self.loop}")
        acts, cost = plan_route(SPAWN_CELL, 0, self.pellets, self.near_ghost, self.ghost_edges(),
                                no_arc=self.no_arc)
        if not acts:
            self.actions = []
            return
        n_f = sum(v for k, v in acts if k == "run")
        n_t = sum(1 for k, v in acts if k == "turn")
        self.log(f"pellets {self.pellets}")
        self.log(f"plan    {n_f} forward moves, {n_t} turns, about {cost:.0f} s")
        self.log(f"route   {route_string(acts, 0)}")
        # We start in the chute WEST of (14, 0): the first action drives in.
        if acts[0][0] == "run":
            acts[0] = ("run", acts[0][1] + 1)
        else:
            acts.insert(0, ("run", 1))
        self.cell, self.dir = (SPAWN_CELL[0], -1), 0      # virtual cell in the chute
        acts = make_arcs(acts, self.cell, 0, self.pellets, self.no_arc)
        n_arc = sum(1 for k, v in acts if k == "arc")
        self.log(f"corners: {n_arc} driven as arcs, {n_t - n_arc} stop-and-pivot "
                 f"(route letters: lower case = arc)")
        self.log(f"route   {route_string(acts, 0)}")
        self.actions = acts
        self.th = 0.0
        self.x = SPAWN_XY[0] - AXLE_BACK                   # axle is behind the chassis
        self.y = SPAWN_XY[1]
        self.i, self.sub, self.sub_t, self.action_t = 0, None, 0.0, 0.0
        self.chunk_end = 0
        self.next_log = self.t + 10.0
        self.expect_pellets()

    def expect_pellets(self):
        """For each pellet, the index of the action that drives through it."""
        self.expect = {}
        cell, d = self.cell, self.dir
        for k, (kind, val) in enumerate(self.actions):
            if kind in ("turn", "arc"):
                d = val
                continue
            for _ in range(val):
                cell = (cell[0] + STEP[d][0], cell[1] + STEP[d][1])
                if cell in self.pellets and cell not in self.expect:
                    self.expect[cell] = k

    def check_missed(self):
        """A pellet we already drove through is still on the board: replan."""
        missed = [p for p, k in self.expect.items() if k < self.i - 1 and p in self.pellets]
        if missed:
            self.log(f"pellet(s) {missed} still on the board - replanning to collect them")
            self.replan()
            return True
        return False

    def replan(self):
        """Plan afresh from the nearest cell and heading, for the pellets that
        are still on the board."""
        col = int(clamp(round(self.x / PITCH), 0, N - 1))
        row = int(clamp(N - 1 - round(self.y / PITCH), 0, N - 1))
        d = int(round(self.th / (math.pi / 2))) % 4
        acts, cost = plan_route((row, col), d, self.pellets, self.near_ghost, self.ghost_edges(),
                                forbid_ghost=self.give_up_guarded, no_arc=self.no_arc)
        if not acts and self.give_up_guarded is False:
            acts, cost = plan_route((row, col), d, self.pellets, self.near_ghost,
                                    self.ghost_edges(), forbid_ghost=True)
        # Start with a "turn" to the current heading: it re-centres the axle
        # in the cell and squares the robot up before anything else.
        self.actions = make_arcs([("turn", d)] + acts, (row, col), d, self.pellets, self.no_arc)
        self.i, self.cell, self.dir = 0, (row, col), d
        self.sub, self.sub_t, self.action_t = None, 0.0, 0.0
        self.chunk_end = 0
        self.boundary_checked = -1
        self.gated_i = -1
        self.expect_pellets()
        self.log(f"replanned from {(row, col)} facing {d}: {len(self.actions)} actions, "
                 f"pellets left {self.pellets}")

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

    def check_stuck(self, cmd, rate, dt):
        """Look at the last STUCK_WINDOW seconds; return a reason string if
        the robot is clearly not doing what it is being told to."""
        v_cmd = WHEEL_R * (cmd[0] + cmd[1]) / 2
        w_cmd = WHEEL_R * (cmd[1] - cmd[0]) / TRACK
        r = self.last_r
        straight = (self.i < len(self.actions) and self.actions[self.i][0] != "arc"
                    and abs(wrap(self.th - round(self.th / (math.pi / 2)) * math.pi / 2)) < math.radians(8))
        self.hist.append((v_cmd, w_cmd, rate * dt, self.v_meas,
                          {k: r[k][0] for k in r}, {k: r[k][1] for k in r}, straight))
        n = int(STUCK_WINDOW / dt)
        if len(self.hist) > n:
            del self.hist[0]
        if self.t < self.quiet_until or len(self.hist) < n:
            return None
        if self.x < 0.0 or self.x > EXIT_X - 0.12:
            return None                  # in the spawn / exit chutes: walls not on the map
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
        # (Only on straights: in an arc the predictions swing as we rotate.)
        if all(abs(f[0]) > 0.03 and f[6] for f in h):
            for k in ("sl", "sr"):
                dm = abs(h[-1][4][k] - h[0][4][k])
                de = abs(h[-1][5][k] - h[0][5][k])
                vals = [f[4][k] for f in h]
                frozen = max(vals) - min(vals) < 0.004
                agreed = abs(h[0][4][k] - h[0][5][k]) < 0.03     # map matched at the start
                if max(h[-1][4][k], h[0][4][k]) < 1.0 and de > 0.03 and dm < 0.004 and frozen and agreed:
                    return f"{k} reading frozen while moving"
        # The map disagrees with two or more sensors the whole time.
        if all(f[6] and sum(1 for k in f[4] if min(f[4][k], f[5][k]) < 0.4 and abs(f[4][k] - f[5][k]) > 0.04) >= 2
               for f in h):
            return "position estimate disagrees with the walls"
        return None

    def start_recovery(self, why):
        if self.i < len(self.actions) and (self.actions[self.i][0] == "arc" or
                                           (self.i > 0 and self.actions[self.i - 1][0] == "arc")):
            self.arc_scale *= 0.7
            if self.v_arc() == 0.0:
                self.arcs_off = True
                self.log("arcs switched off after trouble in a corner")
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
            return wheel_speeds(rc["dir"] * BACKOFF_SPEED, 0.0)
        if rc["stage"] == "square":
            # Pivot to the nearest axis: the walls are then square-on to the
            # sensors, which makes relocalizing far more reliable.
            d = int(round(self.th / (math.pi / 2))) % 4
            err = wrap(d * math.pi / 2 - self.th)
            if (abs(err) < TURN_TOL and abs(rate) < SETTLE_RATE) or rc["t"] > 3.0:
                rc["stage"], rc["t"] = "settle", 0.0
                return 0.0, 0.0
            w = math.copysign(min(W_MAX, math.sqrt(2 * ALPHA_DEC * abs(err)), K_TURN * abs(err)), err)
            return wheel_speeds(0.0, w)
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
                return wheel_speeds(0.0, w)
            if dist < 0.005 or rc["t"] > 3.0:
                return self.finish_recovery()
            along = dist * math.cos(wrap(bearing - self.th)) * 1.0   # signed distance along heading
            v = clamp(3.0 * along, -0.10, 0.10)
            omega = clamp(K_HEAD * err, -2.0, 2.0) if dist > 0.02 else 0.0
            return wheel_speeds(v, omega)
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

    # ------------------------------------------------------------------ ghost

    def learn_loop(self):
        """Find the ghost's circuit: the smallest k for which its sequence of
        cells repeats every k changes, confirmed over more than one lap."""
        tr = self.g_trans
        n = len(tr)
        cells = [c for _, c in tr]
        for k in range(2, 41):
            checks = n - k - 1                 # skip tr[0]: not a real change
            if checks < max(4, k // 2):
                break
            if all(cells[i] == cells[i + k] for i in range(1, n - k)):
                laps = [tr[i + k][0] - tr[i][0] for i in range(1, n - k)]
                period = sum(laps) / len(laps)
                if period < 1.0 or self.t - tr[1][0] < period:
                    continue
                self.loop = cells[n - k:]
                self.loop_k = k
                self.g_period = period
                self.build_cycle()
                return

    def refine_period(self):
        """Average the lap time over every lap seen so far."""
        tr, k = self.g_trans, self.loop_k
        laps = [tr[i + k][0] - tr[i][0] for i in range(1, len(tr) - k)
                if tr[i][1] == tr[i + k][1]]
        if laps:
            self.g_period = sum(laps) / len(laps)

    def build_cycle(self):
        """One lap of ghost positions, GHOST_BIN apart, taken from the most
        recent lap actually observed (so the prediction never drifts)."""
        self.refine_period()
        P = self.g_period
        times = [t for t, _, _ in self.g_samples]
        self.g_anchor = self.g_samples[-1][0] - P
        nb = max(1, int(round(P / GHOST_BIN)))
        cyc = []
        for b in range(nb):
            t = self.g_anchor + b * P / nb
            j = bisect.bisect_left(times, t)
            j = min(max(j, 0), len(times) - 1)
            cyc.append(self.g_samples[j][1:])
        self.g_cycle = cyc
        near = set()
        for r in range(N):
            for c in range(N):
                cx, cy = cell_centre((r, c))
                if min(math.hypot(cx - gx, cy - gy) for gx, gy in cyc) < SAFE_CELL_DIST:
                    near.add((r, c))
        self.near_ghost = frozenset(near)
        self.no_arc = frozenset(
            (r, c) for r in range(N) for c in range(N)
            if min(math.hypot(cell_centre((r, c))[0] - gx, cell_centre((r, c))[1] - gy)
                   for gx, gy in cyc) < ARC_GHOST_CLEAR)

    def ghost_edges(self):
        """Pairs (a, b) of loop cells the ghost moves from a to b."""
        if not self.loop:
            return frozenset()
        L = self.loop
        return frozenset((L[i], L[(i + 1) % len(L)]) for i in range(len(L)))

    def ghost_dist(self, x, y):
        """Closest the ghost's path ever comes to the point (x, y)."""
        return min(math.hypot(x - gx, y - gy) for gx, gy in self.g_cycle)

    def timeline(self):
        """Where the axle will be, every 0.1 s, if the plan runs from now:
        from the current action up to the next stop at a cell the ghost can
        never reach (or out of the maze). Returns (samples, end index)."""
        tl = []
        tau = 0.0
        x, y = self.x, self.y
        cell, d = self.cell, self.dir
        k = self.i
        kt = self.k_time

        def add(x0, y0, x1, y1, dur):
            nonlocal tau
            n = max(1, int(dur / 0.1))
            for j in range(n):
                f = (j + 1) / n
                tl.append((tau + dur * f, x0 + f * (x1 - x0), y0 + f * (y1 - y0)))
            tau += dur

        while k < len(self.actions):
            kind, val = self.actions[k]
            if kind == "arc":
                cx, cy = cell_centre(cell)
                add(x, y, cx, cy, kt * T_ARC)
                x, y, d = cx, cy, val
                k += 1
                continue
            if kind == "turn":
                cx, cy = cell_centre(cell)
                q = 2 if (val - d) % 4 == 2 else 1
                align = math.hypot(cx - x, cy - y)
                add(x, y, cx, cy, kt * (align / 0.1))
                add(cx, cy, cx, cy, kt * (T_TURN_FIXED + T_TURN_Q * q))
                x, y, d = cx, cy, val
                k += 1
                continue
            tgt = (cell[0] + STEP[d][0] * val, cell[1] + STEP[d][1] * val)
            tx, ty = cell_centre(tgt)
            add(x, y, tx, ty, kt * (val * PITCH / V_MAX + T_RUN_EXTRA))
            x, y, cell = tx, ty, tgt
            k += 1
            if k >= len(self.actions) or not inside(*cell):
                break                               # out of the maze
            if self.ghost_dist(tx, ty) >= SAFE_CELL_DIST:
                break                               # a safe place to stop
        return tl, k

    def gate(self):
        """Before the next stretch: find the earliest moment to set off so
        that the ghost - predicted from its learned lap, with a timing margin
        that grows the further ahead we look - stays GHOST_SAFE away the whole
        way, including while we wait here. Returns the wait in seconds."""
        if self.loop is None:
            self.chunk_end = len(self.actions)
            return 0.0
        self.build_cycle()
        tl, end = self.timeline()
        self.chunk_end = end
        cyc, P = self.g_cycle, self.g_period
        nb = len(cyc)
        bin_t = P / nb

        def prefix(px, py):
            flags = [1 if math.hypot(px - gx, py - gy) < GHOST_SAFE else 0 for gx, gy in cyc]
            if not any(flags):
                return None
            pre = [0]
            for f in flags + flags:
                pre.append(pre[-1] + f)
            return pre

        def hits(pre, t0, t1):
            b0 = math.floor((t0 - self.g_anchor) / bin_t)
            b1 = math.ceil((t1 - self.g_anchor) / bin_t)
            if b1 - b0 >= nb:
                return pre[nb] > 0
            m = b0 % nb
            return pre[m + (b1 - b0) + 1] - pre[m] > 0

        near = [(tau, pre) for tau, x, y in tl for pre in [prefix(x, y)] if pre]
        here = prefix(self.x, self.y)
        if not near and not here:
            return 0.0
        best, best_bad = 0.0, None
        w = 0.0
        while w <= P + 10.0:
            bad = 0
            if here and w > 0 and hits(here, self.t, self.t + w + 0.3):
                bad += 5
            for tau, pre in near:
                slack = SLACK_BASE + SLACK_FRAC * tau
                if hits(pre, self.t + w + tau - slack, self.t + w + tau + slack):
                    bad += 1
            if bad == 0:
                best, best_bad = w, 0
                break
            if best_bad is None or bad < best_bad:
                best, best_bad = w, bad
            w += WAIT_STEP
        if best_bad:
            # No moment is safe for this stretch: the ghost would be met
            # whenever we went. Never drive into it knowingly - the run would
            # end on the spot. Skip what is only reachable through its loop.
            if not self.give_up_guarded and not here:
                self.log(f"ghost: no safe moment for actions {self.i}-{end - 1} "
                         f"({best_bad} close calls at best) - skipping what lies on its loop")
                self.give_up_guarded = True
                self.replan()
                return 0.05 + 1e-3
            self.log(f"ghost: no fully safe moment, going at the least risky ({best_bad} close calls)")
        if best > 0.05:
            self.log(f"ghost nearby: waiting {best:.1f} s before actions {self.i}-{end - 1}")
        return best

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
            nxt = self.actions[self.i + 1][0] if self.i + 1 < len(self.actions) else None
            if nxt == "arc" and self.action_t <= dt:
                v = self.v_arc()
                if v == 0.0:
                    # Not safe (yet): drive this corner as a stop-and-pivot.
                    self.actions[self.i + 1] = ("turn", self.actions[self.i + 1][1])
                    nxt = "turn"
                else:
                    self.arc_v = v
            if nxt == "arc":
                # Flow into the corner: aim for where the turn cell begins and
                # arrive there at V_ARC, without stopping.
                tx, ty = cell_centre(target)
                ux, uy = math.cos(self.dir * math.pi / 2), math.sin(self.dir * math.pi / 2)
                cmd, rem = self.drive_line((tx - ARC_R * ux, ty - ARC_R * uy), self.dir,
                                           stop=False, v_end=self.arc_v)
                if rem <= 0.0:
                    self.cell = target
                    self.next_action()
                return cmd
            cmd, arrived = self.drive_line(cell_centre(target), self.dir, stop=True)
            if arrived:
                self.cell = target
                self.next_action()
            return cmd

        if kind == "arc":
            return self.arc(val, dt)

        # kind == "turn": first put the axle on the cell centre, then pivot.
        if self.sub is None:
            self.sub, self.sub_t = "align", 0.0
            self.turn_q = 2 if (val - self.dir) % 4 == 2 else 1
        if self.sub == "align":
            cmd, arrived = self.drive_line(cell_centre(self.cell), self.dir, stop=True)
            if arrived or self.sub_t > 3.0:
                self.sub, self.sub_t = "pivot", 0.0
                self.v = 0.0
            return cmd
        err = wrap(val * math.pi / 2 - self.th)
        if self.sub_t <= dt:                      # first pivot frame
            self.turn_sign = 1.0 if err >= 0 else -1.0
            self.overshoot = 0.0
            self.spin = []
        if len(self.spin) < 50:
            # How fast can the robot build up a turn? (traction-limited)
            # Look at the first 50 frames (0.5 s) of the pivot.
            self.spin.append(abs(rate))
            if len(self.spin) == 50 and abs(err) > 0.3:
                rates = self.spin
                best = max((rates[j + 5] - rates[j]) / (5 * dt) for j in range(len(rates) - 5))
                if best > 0.5:
                    self.alpha_meas.append(best)
                    if len(self.alpha_meas) == ARC_LEARN_PIVOTS:
                        self.log(f"turn acceleration about {best:.1f} rad/s^2 - "
                                 f"arc corners at {self.v_arc():.2f} m/s"
                                 if self.v_arc() else
                                 f"turn acceleration about {best:.1f} rad/s^2 - too low for arcs")
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
        return wheel_speeds(0.0, w)

    def v_arc(self):
        """Arc speed for the turn acceleration this robot has shown, or 0 if
        arcs are not (yet) safe."""
        if self.arcs_off or len(self.alpha_meas) < ARC_LEARN_PIVOTS:
            return 0.0
        a = sorted(self.alpha_meas)[len(self.alpha_meas) // 2]       # median
        v = min(V_ARC, ARC_SWING * a * ARC_R) * self.arc_scale
        return v if v >= V_ARC_MIN else 0.0

    def arc(self, d1, dt):
        """Drive a quarter-circle of radius ARC_R through the current cell,
        from the edge we entered by to the edge we leave by, round the inside
        corner post. Feedforward turn rate v / R, corrected for heading and
        radius errors; done once the exit edge is crossed."""
        d0 = self.dir
        s = 1.0 if (d1 - d0) % 4 == 1 else -1.0          # +1 left, -1 right
        cx, cy = cell_centre(self.cell)
        ux, uy = math.cos(d0 * math.pi / 2), math.sin(d0 * math.pi / 2)
        wx, wy = math.cos(d1 * math.pi / 2), math.sin(d1 * math.pi / 2)
        ox, oy = cx - ARC_R * ux + ARC_R * wx, cy - ARC_R * uy + ARC_R * wy   # corner post
        ex, ey = cx + ARC_R * wx, cy + ARC_R * wy                             # exit point
        rx, ry = self.x - ox, self.y - oy
        r = math.hypot(rx, ry)
        th_des = math.atan2(ry, rx) + s * math.pi / 2     # tangent to the circle
        e_h = wrap(th_des - self.th)
        e_r = r - ARC_R                                   # >0: too far out
        step = ACCEL * 0.01
        self.v += clamp(self.arc_v - self.v, -step, step)
        omega = s * self.v / ARC_R + K_ARC_H * e_h + s * K_ARC_R * e_r
        self.arc_err = max(self.arc_err, abs(e_r))
        omega = clamp(omega, -W_MAX, W_MAX)
        past = (self.x - ex) * wx + (self.y - ey) * wy >= 0.0
        if past or self.action_t > 4.0:
            self.n_arcs += 1
            if self.arc_err > ARC_BAD_R:
                self.arc_scale *= 0.8
                self.log(f"arc strayed {self.arc_err * 100:.1f} cm off its circle - "
                         f"slowing arcs to {self.v_arc():.2f} m/s")
                if self.v_arc() == 0.0:
                    self.arcs_off = True
                    self.log("arcs switched off: every corner will stop and pivot")
            self.arc_err = 0.0
            self.dir = d1
            self.next_action()
        return wheel_speeds(self.v, omega)

    def next_action(self):
        # Learn how long actions really take compared with the time model -
        # only from stop-to-stop runs and pivots (arcs flow, so they differ).
        kind, val = self.actions[self.i]
        nxt = self.actions[self.i + 1][0] if self.i + 1 < len(self.actions) else None
        prv = self.actions[self.i - 1][0] if self.i > 0 else None
        if kind == "arc" or nxt == "arc" or prv == "arc":
            self.i += 1
            self.sub, self.sub_t, self.action_t = None, 0.0, 0.0
            return
        if kind == "run":
            model = val * PITCH / V_MAX + T_RUN_EXTRA
        else:
            model = T_TURN_FIXED + T_TURN_Q * getattr(self, "turn_q", 1)
        if self.action_t > 0.3 and model > 0.5:
            ratio = self.action_t / model
            self.k_time = clamp(0.8 * self.k_time + 0.2 * ratio, 0.7, 2.0)
        self.i += 1
        self.sub, self.sub_t, self.action_t = None, 0.0, 0.0

    def drive_line(self, target, d, stop, v_end=None):
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
        elif v_end is not None:
            vt = min(vmax, math.sqrt(v_end ** 2 + 2 * DECEL * max(rem, 0.0)))
        else:
            vt = vmax
        step = ACCEL * 0.01
        self.v += clamp(vt - self.v, -step, step)
        th_ref = d * math.pi / 2
        if self.v > 0.02:
            # Steer back to the centre line more gently the faster we go
            # (the same correction at higher speed is a sharper swerve).
            k_lat = K_LAT * min(1.0, V_STEER_REF / max(self.v, 1e-3))
            th_ref -= clamp(math.atan(k_lat * lat), -0.35, 0.35)
        omega = clamp(K_HEAD * wrap(th_ref - self.th), -W_STEER, W_STEER)
        cmd = wheel_speeds(self.v, omega)
        if not stop:
            return cmd, rem
        arrived = abs(rem) < STOP_TOL and abs(self.v_meas) < 0.02
        if arrived:
            self.v = 0.0
        return cmd, arrived



# ================================================================ controller

class Controller:
    def __init__(self, maze):
        self.maze = maze
        self.pellets = []     # cells still on the board; filled from MQTT
        self.have_pellets = False
        self.elapsed = 0.0
        self.ghost = None     # latest pacbot/ghost payload, or None
        self.prev_ticks = None  # (left, right) counts from the previous frame
        self.solver = Solver()  # planning, pose tracking, ghost timing, driving
        self.quiet = False

    def on_pellets(self, msg):
        """Called whenever the remaining-pellet list arrives or changes.

        msg is {"pellets": [[row, col], ...]}.
        """
        self.pellets = [tuple(c) for c in msg.get("pellets", [])]
        self.have_pellets = True
        self.solver.quiet = self.quiet
        if not self.quiet:
            print(f"pellets remaining: {self.pellets}", flush=True)
        # The route is planned once the ghost's loop has been watched (before
        # the robot moves, so it costs no time); see Solver.first_plan().
        self.solver.set_pellets(self.pellets)

    def decide(self, s):
        """One sensor frame in, one wheel command out.

        `s` is the decoded pacbot/sensors payload. Return (left, right) in
        rad/s.
        """
        self.elapsed += s["dt"]

        # Nothing to do until we know where the pellets are.
        if not self.have_pellets:
            return 0.0, 0.0

        # The ghost arrives on its own topic every frame; hand the latest
        # position to the solver, then let it drive.
        self.solver.set_ghost(self.ghost)
        return self.solver.step(s)


def main():
    # The official maze, embedded - no file to find, no path to pass.
    # The id printed here must match the one the simulator prints in its
    # banner ("maze official <id> (built in)"). If they ever differ, your
    # planner and the simulator are looking at different walls.
    maze = Maze.official()
    print(f"official maze {maze_2b.MAZE_ID}: {maze.grid}x{maze.grid}, "
          f"spawn row {maze.spawn_row}, exit row {maze.exit_row}")

    controller = Controller(maze)
    client = _mqtt_client()

    def on_message(cli, userdata, msg):
        payload = json.loads(msg.payload.decode())
        if msg.topic == TOPIC_PELLETS:
            controller.on_pellets(payload)
            return
        if msg.topic == TOPIC_GHOST:
            controller.ghost = payload      # {"row":..,"col":..,"x":..,"y":..}
            return
        if msg.topic == TOPIC_RESULT:
            print(f"run over: solved={payload['solved']}, "
                  f"collisions={payload['collisions']}")
            return
        left, right = controller.decide(payload)
        left = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(left)))
        right = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(right)))
        # Key order matters: the simulator matches a fixed pattern. See the
        # pacbot/wheel_vel notes in the header comment.
        cli.publish(TOPIC_WHEEL_VEL,
                    json.dumps({"left": left, "right": right}))

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_PELLETS)
    client.subscribe(TOPIC_GHOST)
    client.subscribe(TOPIC_RESULT)
    client.subscribe(TOPIC_SENSORS)
    print("connected; waiting for the simulator")
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass