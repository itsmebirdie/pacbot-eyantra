"""Boilerplate for PB Task 3B.

All the plumbing, none of the thinking. It connects to the broker, loads the
official maze, receives the pellet positions for this run, and publishes a
wheel velocity command on every sensor frame. Your job is the one TODO block
in `decide()`.

Run it in two terminals, from the folder you unpacked:

    ./task_3b_launch                 # terminal 1: the simulator
    python3 task_3b_boilerplate.py   # terminal 2: this file

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
you already parsed, from a copy embedded in maze_3b.py, which is the same text
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
             +---+---+---+---+---+---+---+ E +---+---+---+---+---+---+---+
     row 0   |                           |               |               |
             +   +---+   +   +---+   +   +   +---+---+   +   +---+   +   +
     row 1   |           |   |       |           |                   |   |
             +   +---+---+   +   +   +   +---+   +   +   +---+---+   +   +
     row 2   |       |       |   |       |           |       |G2         |
             +---+   +   +---+   +---+   +   +---+---+   +   +   +---+   +
     row 3   |       |   |       |                       |               |
             +   +---+   +   +---+   +---+   +---+   +---+   +---+   +---+
     row 4   |               |                       |           |       |
             +   +   +---+   +   +---+---+---+   +   +   +---+   +   +   +
     row 5   |   |               |               |       |       |   |   |
             +   +   +---+   +   +   +   +---+---+---+   +   +---+   +   +
     row 6   |   |           |       |   |                   |           |
             +   +---+---+   +---+   +   +   +---+---+   +   +   +---+   +
     row 7   |   |       |           |   |G1     |       |   |           E
             +   +   +   +   +---+   +   +   +   +   +   +   +   +   +---+
     row 8   |       |                   |   |       |           |       |
             +---+   +   +---+---+---+   +   +---+---+---+---+   +---+   +
     row 9   |       |                               |           |       |
             +   +   +   +   +---+   +---+   +---+   +   +---+   +   +   +
     row 10  |   |       |           |                       |       |   |
             +   +---+   +---+---+   +   +---+   +---+---+   +   +   +   +
     row 11  |   |                           |           |       |       |
             +   +   +---+---+   +---+   +   +---+---+   +---+   +---+   +
     row 12  |       |                   |           |       |           |
             +---+   +   +---+---+---+   +   +   +   +---+   +---+   +   +
     row 13  |           |           |       |   |   |               |   |
             +   +---+   +   +   +   +   +   +   +   +   +   +---+---+   +
     row 14  S                   |       |               |               |
             +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

You START outside the west border at the `S` gap on row 14, facing EAST, and
drive into cell (14, 0). You FINISH by leaving through ANY of the `E` gaps -
every exit counts the same, so pick whichever suits your route and the
ghosts. `maze.exits` lists them. Leaving through an exit is the only thing
that ends a run successfully.

There are no dead ends: every cell has at least two ways out.

There are TWO ghosts. `G1` marks where ghost 1 starts, (7, 7), and `G2`
where ghost 2 starts, (2, 12). They start there every run.

Coordinates. Cells are (row, col), both zero-based, with row 0 at the TOP of
the drawing and col 0 at the LEFT. Headings are

    EAST = 0    NORTH = 1    WEST = 2    SOUTH = 3

The order runs counter-clockwise, so (heading + 1) % 4 is a 90 degree turn to
the LEFT, and (heading + 3) % 4 is a turn to the RIGHT. You spawn facing EAST,
which is heading 0.

Cell centres in metres. This is the frame the ghosts' x/y arrive in, and the
one to dead-reckon in:

    x = col * 0.22
    y = (14 - row) * 0.22

So cell (14, 0) sits at (0.00, 0.00) and cell (0, 14) at (3.08, 3.08). Driving
EAST increases x; driving NORTH increases y. Going back the other way:

    col = round(x / 0.22)
    row = 14 - round(y / 0.22)

You spawn at (-0.15, 0.00), in the short chute outside the west border, so
your first move is eastward through the gap. The run ends when your chassis
passes an exit's border plane, half a pitch beyond the outer cell centres:
x = 3.19 for an east exit, y = 3.19 for a north one.

Asking the maze about walls, rather than working it out from the drawing:

    maze.can_move((7, 0), Maze.EAST)     # -> True / False
    maze.grid                            # 15
    maze.pitch                           # 0.22
    maze.spawn_row                       # 14
    maze.exits                           # [Exit(east, 7), Exit(north, 7)]
    maze.exit_cell(maze.exits[0])        # (7, 14): the cell you leave from
    maze.ghost_homes                     # [(7, 7), (2, 12)]: ghost 1, ghost 2

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
  pacbot/ghost2
    {"row": .., "col": .., "x": .., "y": ..}

    Where each ghost is RIGHT NOW: its cell and its exact position in
    metres. pacbot/ghost is ghost 1, pacbot/ghost2 is ghost 2; the payload is
    the same for both. RETAINED, so subscribing late still tells you.

    The ghosts move around the maze, and touching EITHER of them ends your
    run at once. Ghost 1 starts at (7, 7) and ghost 2 at (2, 12), every run.
    Watch where they go.

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
leaves through any exit, or either ghost comes within 0.11 m of it.
"""
import heapq
import itertools
import json
import math
import os

import paho.mqtt.client as mqtt

import maze_3b
from maze_3b import Maze

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_PELLETS = "pacbot/pellets"
TOPIC_GHOST = "pacbot/ghost"
TOPIC_GHOST2 = "pacbot/ghost2"
TOPIC_RESULT = "pacbot/result"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

WHEEL_RADIUS = 0.017  # m
HALF_TRACK = 0.039    # m, wheel centre to centreline
WHEEL_LIMIT = 30.0    # rad/s, the simulator clamps to this
TICKS_PER_REV = 360   # encoder counts per wheel revolution
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV   # radians of wheel turn per tick

# ----------------------------------------------------------------------------
# Our own constants
# ----------------------------------------------------------------------------
PITCH = 0.22            # m, cell pitch
AXLE_OFFSET = 0.033     # m, axle sits this far behind the chassis centre
SPAWN_X = -0.15         # m, chassis centre at spawn (facing east)
SIDE_CLEAR = 0.10       # m, side ToF reading when centred in a corridor
FRONT_OFFSET = 0.0      # m, front ToF reading minus distance to wall line

FRONT_BASELINE = 0.064  # m, lateral spacing of the two forward ToF rays
FRONT_LAT = 0.032       # m, each forward ray sits this far off the centreline
SIDE_FACE = 0.01        # m, side rays read to the wall face (half thickness)
TOF_MAX = 2.0           # m, ToF range limit
MATCH_SPAN = 24         # along-track candidates each side, MATCH_STEP apart
MATCH_STEP = 0.005      # m
ENCODER_SCALE = 0.98    # true distance / encoder distance (wheel slip)
LATENCY = 0.04          # s, a wheel command takes this long to act

# Driving
V_MAX = 0.48            # m/s cruise speed on straights
A_ACC = 2.0             # m/s^2 acceleration limit (harder spins the wheels)
A_DEC = 3.0             # m/s^2 braking used for the stop profile
STOP_BIAS = 0.008       # m, stop the axle this far short of the cell centre
W_MAX = 2.5             # rad/s heading-correction limit while driving
TURN_GAIN = 40.0        # wheel rad/s per rad of turn error (pivot)
TURN_DONE = 0.05        # rad, pivot finishes inside this error
TURN_MIN = 10.0         # wheel rad/s, below this the pivot stalls on friction
TURN_ALPHA = 60.0       # rad/s^2, how hard a pivot spins down when released
K_LAT = 9.0             # rad of heading correction per m of lateral error
K_HEAD = 7.0            # 1/s heading loop gain

# Planning (seconds)
T_CELL = PITCH / 0.46   # one cell at cruise
T_TURN90 = 0.90         # stop + 90 deg pivot + restart (measured)
T_TURN180 = 1.20        # stop + 180 deg pivot + restart (measured)
T_EXIT = 0.4            # leaving through the exit gap

# Ghost
GHOST_SPEED = 0.168     # m/s, both ghosts: one cell per 1.33 s (0.165), plus margin
SAFETY_BUFFER = 1.4     # s, must clear a cell this long before either ghost
PLAN_HORIZON = 12.0     # s, how far ahead the ghost constraint applies
SOFT_MARGIN = 3.0       # s, margins below this add a soft cost
SOFT_WEIGHT = 0.4       # s of cost per s of margin below SOFT_MARGIN
FLEE_DEPTH = 3.0        # s, a flee route must look safe at least this far

# Escaping, when no route keeps the safety buffer: every short escape path is
# simulated with both ghosts chasing the bot along shortest paths (ghost 1
# does this every step, ghost 2 nearly every step), and the path that stays
# furthest from them wins.
FLEE_T = 5.0            # s, length of the escape paths tried
FLEE_DT = 0.2           # s, simulation step
FLEE_MAX_PATHS = 600    # cap on paths simulated per decision
FLEE_END_W = 0.3        # weight on the room left at the end of the path
FRONT_STOP = 0.080      # m, front reading of a bot stopped on a cell centre
SECOND_BUFFER = 0.7     # s, reduced buffer tried before escaping

DEBUG = bool(os.environ.get("PACBOT_DEBUG"))

HEADING_ANGLE = (0.0, math.pi / 2, math.pi, -math.pi / 2)
TURN_NAME = {1: "L", 3: "R", 2: "U"}


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def wheels(forward, turn):
    """Differential drive mixer: +turn steers LEFT (counter-clockwise)."""
    return forward - turn, forward + turn


# ----------------------------------------------------------------------------
# Planner: time-weighted search over (cell, heading) states
# ----------------------------------------------------------------------------
class Planner:
    """Shortest-time routes over (cell, heading) states.

    A state is a cell plus the heading the bot faces in it. Moving one cell
    forward costs T_CELL; turning in place costs T_TURN90 / T_TURN180, which
    include the stop and restart. The ghost is handled as a hard constraint
    inside the search: a state may only be used if the bot is there at least
    SAFETY_BUFFER seconds before the ghost could possibly arrive.
    """

    def __init__(self, maze):
        self.maze = maze
        self.n = maze.grid
        self.size = self.n * self.n * 4
        self.edges = [[] for _ in range(self.size)]
        self.cell_adj = [[] for _ in range(self.n * self.n)]
        for r in range(self.n):
            for c in range(self.n):
                for h in range(4):
                    u = self.node((r, c), h)
                    if maze.can_move((r, c), h):
                        dr, dc = Maze.DELTA[h]
                        self.edges[u].append((self.node((r + dr, c + dc), h), T_CELL))
                        self.cell_adj[r * self.n + c].append((r + dr) * self.n + c + dc)
                    self.edges[u].append((self.node((r, c), (h + 1) % 4), T_TURN90))
                    self.edges[u].append((self.node((r, c), (h + 3) % 4), T_TURN90))
                    self.edges[u].append((self.node((r, c), (h + 2) % 4), T_TURN180))
        self.exits = [(maze.exit_cell(e), e.side) for e in maze.exits]
        self._static = {}
        # Corridor distance (in cells) between every pair of cells.
        cells = self.n * self.n
        self.hops = []
        for src in range(cells):
            d = [-1] * cells
            d[src] = 0
            queue = [src]
            for idx in queue:
                for j in self.cell_adj[idx]:
                    if d[j] < 0:
                        d[j] = d[idx] + 1
                        queue.append(j)
            self.hops.append(d)

    def node(self, cell, h):
        return ((cell[0] * self.n + cell[1]) << 2) | h

    def cell_of(self, node):
        idx = node >> 2
        return (idx // self.n, idx % self.n)

    # -- searches ------------------------------------------------------------
    def dijkstra(self, src, t0=0.0, eta=None, buf=0.0, no_turn_src=False):
        """Earliest-arrival search. With `eta`, states the ghost could reach
        within `buf` seconds of us are forbidden, and close passes cost extra.
        Returns (cost, prev); cost is time plus the soft ghost penalty."""
        cost = [math.inf] * self.size
        time_at = [math.inf] * self.size
        prev = [-1] * self.size
        cost[src] = time_at[src] = t0
        heap = [(t0, src)]
        while heap:
            c, u = heapq.heappop(heap)
            if c > cost[u]:
                continue
            t = time_at[u]
            for v, step in self.edges[u]:
                if no_turn_src and u == src and (v >> 2) == (u >> 2):
                    continue
                nt = t + step
                nc = c + step
                if eta is not None and nt <= PLAN_HORIZON:
                    margin = eta[v >> 2] - nt
                    if margin < buf:
                        continue
                    if margin < SOFT_MARGIN:
                        nc += SOFT_WEIGHT * (SOFT_MARGIN - margin) * step
                if nc >= cost[v]:
                    continue
                cost[v] = nc
                time_at[v] = nt
                prev[v] = u
                heapq.heappush(heap, (nc, v))
        return cost, prev

    def flee(self, src, eta):
        """Widest-path search: the route whose closest ghost approach (in
        seconds of margin) is as large as possible. Used when no route
        satisfies the hard safety buffer."""
        best = [-math.inf] * self.size
        time_at = [math.inf] * self.size
        prev = [-1] * self.size
        best[src] = math.inf
        time_at[src] = 0.0
        heap = [(-math.inf, 0.0, src)]
        while heap:
            negm, t, u = heapq.heappop(heap)
            m = -negm
            if m < best[u] or t > FLEE_DEPTH + 1.0:
                continue
            for v, step in self.edges[u]:
                nt = t + step
                nm = min(m, eta[v >> 2] - nt)
                if nm > best[v] or (nm == best[v] and nt < time_at[v]):
                    best[v] = nm
                    time_at[v] = nt
                    prev[v] = u
                    heapq.heappush(heap, (-nm, nt, v))
        target, score = src, -math.inf
        for v in range(self.size):
            if v != src and time_at[v] >= FLEE_DEPTH and best[v] > score:
                score, target = best[v], v
        if target == src:
            for v in range(self.size):
                if v != src and best[v] > score:
                    score, target = best[v], v
        path = [target]
        while path[-1] != src and path[-1] >= 0:
            path.append(prev[path[-1]])
        path.reverse()
        return path, score

    # -- chase-aware escape ------------------------------------------------------
    def _step_time(self, a, b):
        if (a >> 2) != (b >> 2):
            return T_CELL
        return T_TURN180 if ((a & 3) - (b & 3)) % 4 == 2 else T_TURN90

    def _corridor_gap(self, ga, gb, gf, ba, bb, bf):
        """Corridor distance (m) between a ghost on edge ga->gb (fraction gf
        along it) and the bot on edge ba->bb (fraction bf)."""
        if ga == bb and gb == ba:
            return abs(1.0 - gf - bf) * PITCH
        if ga == ba and gb == bb:
            return abs(gf - bf) * PITCH
        best = math.inf
        hops = self.hops
        for ge, gd in ((ga, gf), (gb, 1.0 - gf)):
            row = hops[ge]
            for be, bd in ((ba, bf), (bb, 1.0 - bf)):
                h = row[be]
                if h >= 0:
                    d = (gd + h + bd) * PITCH
                    if d < best:
                        best = d
        return best

    def _chase(self, route, ghost_states, t_min=0.0):
        """Simulate the bot along `route` (starting now) with every ghost
        chasing it; if the route is shorter than `t_min` the bot then waits at
        its end. Returns (closest corridor gap, gap at the end), metres."""
        legs = []
        t = 0.0
        for a, b in zip(route, route[1:]):
            dt = self._step_time(a, b)
            legs.append((t, t + dt, a >> 2, b >> 2))
            t += dt
        t_end = max(t, t_min)
        ghosts = [list(g) for g in ghost_states]
        g_step = GHOST_SPEED * FLEE_DT / PITCH
        adj, hops = self.cell_adj, self.hops
        worst = math.inf
        last = math.inf
        k = 0
        tt = 0.0
        here0 = route[0] >> 2
        while True:
            while k < len(legs) and legs[k][1] < tt:
                k += 1
            if k >= len(legs):
                ba = bb = legs[-1][3] if legs else here0
                bf = 0.0
            else:
                ts, te, ba, bb = legs[k]
                bf = 0.0 if ba == bb else min(1.0, (tt - ts) / (te - ts))
            bot_cell = bb if bf >= 0.5 else ba
            last = math.inf
            for g in ghosts:
                gap = self._corridor_gap(g[0], g[1], g[2], ba, bb, bf)
                if gap < last:
                    last = gap
                g[2] += g_step
                while g[2] >= 1.0 or g[0] == g[1]:
                    if g[0] != g[1]:
                        g[2] -= 1.0
                    here = g[1]
                    d_here = hops[here][bot_cell]
                    nxt = here
                    for j in adj[here]:
                        if hops[j][bot_cell] == d_here - 1:
                            nxt = j
                            break
                    g[0], g[1] = here, nxt
                    if nxt == here:
                        g[2] = 0.0
                        break
            if last < worst:
                worst = last
            if tt >= t_end:
                break
            tt = min(tt + FLEE_DT, t_end)
        return worst, last

    def ghost_states(self, ghosts, motions):
        """Ghost payloads plus their recent motion -> [from, to, frac] edges
        (cell indices), so the simulation starts each ghost moving the way it
        really is."""
        out = []
        n = self.n
        for g, mv in zip(ghosts, motions):
            if g is None:
                continue
            r = min(max(int(g["row"]), 0), n - 1)
            c = min(max(int(g["col"]), 0), n - 1)
            here = r * n + c
            cx, cy = self.maze.cell_center((r, c))
            ox, oy = g["x"] - cx, g["y"] - cy
            state = [here, here, 0.0]
            if mv is not None and math.hypot(*mv) > 1e-4:
                mx, my = mv
                if abs(mx) >= abs(my):
                    h, off = (0 if mx > 0 else 2), ox
                else:
                    h, off = (1 if my > 0 else 3), oy
                along = off if h in (0, 1) else -off      # + means past centre
                dr, dc = Maze.DELTA[h]
                if along >= 0 and self.maze.can_move((r, c), h):
                    state = [here, (r + dr) * n + c + dc, along / PITCH]
                elif along < 0 and self.maze.can_move((r, c), (h + 2) % 4):
                    state = [(r - dr) * n + c - dc, here, 1.0 + along / PITCH]
            out.append(state)
        return out

    def chase_flee(self, start, ghost_states, just_turned=False):
        """Try every escape path of about FLEE_T seconds (no revisited cells,
        no two turns in a row) and return the one that keeps the most room
        from the ghosts while they chase, then ends furthest from them."""
        paths = []
        # Having just turned here, the escape must drive off, not turn again
        # (unless the way ahead is a wall): that is what stops dithering.
        can_go = any((v >> 2) != (start >> 2) for v, _ in self.edges[start])
        stack = [([start], 0.0, just_turned and can_go, {start >> 2})]
        while stack and len(paths) < FLEE_MAX_PATHS:
            path, t, turned, seen = stack.pop()
            u = path[-1]
            grew = False
            if t < FLEE_T:
                for v, step in self.edges[u]:
                    is_turn = (v >> 2) == (u >> 2)
                    if is_turn and turned:
                        continue
                    if is_turn and not any((w >> 2) != (v >> 2) for w, _ in self.edges[v]):
                        continue            # never turn to face a wall
                    if not is_turn and (v >> 2) in seen:
                        continue
                    nseen = seen if is_turn else seen | {v >> 2}
                    stack.append((path + [v], t + step, is_turn, nseen))
                    grew = True
            if not grew and len(path) > 1:
                paths.append(path)
        best, best_key = None, None
        for path in paths:
            worst, last = self._chase(path, ghost_states, FLEE_T)
            key = (worst + FLEE_END_W * last, worst)
            if best_key is None or key > best_key:
                best, best_key = path, key
        if best is None:
            return [start], -math.inf
        return best, best_key[0]

    def static_from(self, node):
        if node not in self._static:
            self._static[node] = self.dijkstra(node)[0]
        return self._static[node]

    def ghost_eta(self, ghosts):
        """Earliest time (s) any ghost could reach each cell, from now."""
        n = self.n
        eta = [math.inf] * (n * n)
        for ghost in ghosts:
            if ghost is not None:
                eta = [min(a, b) for a, b in zip(eta, self._one_ghost_eta(ghost))]
        return eta

    def _one_ghost_eta(self, ghost):
        n = self.n
        gr = min(max(int(ghost["row"]), 0), n - 1)
        gc = min(max(int(ghost["col"]), 0), n - 1)
        cx, cy = self.maze.cell_center((gr, gc))
        off = math.hypot(ghost["x"] - cx, ghost["y"] - cy)
        hops = [-1] * (n * n)
        start = gr * n + gc
        hops[start] = 0
        queue = [start]
        for idx in queue:
            for j in self.cell_adj[idx]:
                if hops[j] < 0:
                    hops[j] = hops[idx] + 1
                    queue.append(j)
        return [math.inf if d < 0 else max(0.0, d * PITCH - off) / GHOST_SPEED
                for d in hops]

    # -- route choice ----------------------------------------------------------
    def _rest_cost(self, node, order, exit_nodes):
        """Static (ghost-free) time from `node` through `order`, then out."""
        best = {node: 0.0}
        for p in order:
            nxt = {}
            for u, t in best.items():
                d = self.static_from(u)
                for h in range(4):
                    v = self.node(p, h)
                    if t + d[v] < nxt.get(v, math.inf):
                        nxt[v] = t + d[v]
            best = nxt
        total = math.inf
        for u, t in best.items():
            d = self.static_from(u)
            for v in exit_nodes:
                total = min(total, t + d[v])
        return total

    def plan(self, cell, heading, t0, pellets, ghosts, just_turned=False,
             motions=None):
        """Return (action, path, cost). action is F/L/R/U/X.

        `just_turned` means we have just pivoted in this cell: turning again
        is only allowed if driving on is unsafe, so the plan cannot dither.
        """
        start = self.node(cell, heading)
        pellets = [p for p in pellets if tuple(p) != tuple(cell)]
        exit_nodes = [self.node(c, h) for c, h in self.exits]
        if not pellets and start in exit_nodes:
            return "X", [start], 0.0
        eta = self.ghost_eta(ghosts)
        tries = [(SAFETY_BUFFER, False)]
        if SECOND_BUFFER > 0:
            tries.append((SECOND_BUFFER, False))
        if just_turned and self.maze.can_move(cell, heading):
            tries.insert(0, (SAFETY_BUFFER, True))
        for buf, no_turn in tries:
            dist, prev = self.dijkstra(start, t0, eta, buf, no_turn_src=no_turn)
            best_cost, best_target = math.inf, None
            if pellets:
                for order in itertools.permutations(pellets):
                    first, rest = order[0], order[1:]
                    for h in range(4):
                        v = self.node(first, h)
                        if dist[v] == math.inf:
                            continue
                        cost = dist[v] + self._rest_cost(v, rest, exit_nodes)
                        if cost < best_cost:
                            best_cost, best_target = cost, v
            else:
                for v in exit_nodes:
                    if dist[v] < best_cost:
                        best_cost, best_target = dist[v], v
            if best_target is not None:
                break
        if best_target is None:
            # No route keeps a safe margin: escape, simulating the chase.
            gstates = self.ghost_states(ghosts, motions or [None] * len(ghosts))
            path, score = self.chase_flee(start, gstates, just_turned)
            best_cost = math.inf
            if DEBUG:
                print(f"    FLEE gap {score:.2f}m via {[self.cell_of(n) for n in path[:6]]}")
        else:
            path = [best_target]
            while path[-1] != start:
                path.append(prev[path[-1]])
            path.reverse()
        if len(path) == 1:
            return "F", path, best_cost
        nxt = path[1]
        if (nxt >> 2) == (start >> 2):
            return TURN_NAME[((nxt & 3) - heading) % 4], path, best_cost
        return "F", path, best_cost


# ----------------------------------------------------------------------------
# Controller
# ----------------------------------------------------------------------------
class Controller:
    def __init__(self, maze):
        self.maze = maze
        self.pellets = []     # cells still on the board; filled from MQTT
        self.have_pellets = False
        self.elapsed = 0.0
        self.ghost = None     # latest pacbot/ghost payload (ghost 1), or None
        self.ghost2 = None    # latest pacbot/ghost2 payload (ghost 2), or None
        self.prev_ticks = None  # (left, right) counts from the previous frame

        self.planner = Planner(maze)
        # Pose of the AXLE midpoint in the world frame.
        self.x = SPAWN_X - AXLE_OFFSET
        self.y = maze.cell_center(maze.spawn_cell)[1]
        self.th = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.mode = "WAIT"            # WAIT, STRAIGHT, TURN, EXIT
        self.cell = maze.spawn_cell   # node we are driving to / standing on
        self.heading = Maze.EAST
        self.action = "F"
        self.path = []
        self.turn_target = 0
        self.speed = 0.0
        self.yaw_f = 0.0
        self.turn_dir = 1
        self.last_s = None
        self._gpos = {}       # ghost index -> recent (x, y) samples

    # ------------------------------------------------------------ callbacks
    def on_pellets(self, msg):
        """Called whenever the remaining-pellet list arrives or changes.

        msg is {"pellets": [[row, col], ...]}.
        """
        self.pellets = [tuple(c) for c in msg.get("pellets", [])]
        self.have_pellets = True
        print(f"pellets remaining: {self.pellets}")

        # Warm the static route cache now, before the robot has moved, so the
        # first plan costs no driving time.
        for p in self.pellets:
            for h in range(4):
                self.planner.static_from(self.planner.node(p, h))

    # ------------------------------------------------------------- helpers
    def _chassis(self):
        return (self.x + AXLE_OFFSET * math.cos(self.th),
                self.y + AXLE_OFFSET * math.sin(self.th))

    def _replan(self, t0, just_turned=False):
        action, path, cost = self.planner.plan(
            self.cell, self.heading, t0, self.pellets, self._ghosts(), just_turned,
            self._ghost_motions())
        self.action, self.path = action, path
        if DEBUG:
            g = [(q["row"], q["col"]) for q in self._ghosts() if q]
            print(f"[{self.elapsed:6.2f}] at {self.cell} h{self.heading} "
                  f"-> {action}  est {cost:5.1f}s  ghost {g}")

    def _ghost_motions(self):
        """How far each ghost moved over the last ~0.2 s, or None."""
        out = []
        for i, g in enumerate(self._ghosts()):
            hist = self._gpos.get(i)
            if g is None or not hist or len(hist) < 2:
                out.append(None)
            else:
                out.append((hist[-1][0] - hist[0][0], hist[-1][1] - hist[0][1]))
        return out

    def _track_ghosts(self):
        for i, g in enumerate(self._ghosts()):
            if g is None:
                continue
            hist = self._gpos.setdefault(i, [])
            hist.append((g["x"], g["y"]))
            if len(hist) > 20:
                del hist[0]

    def _ghosts(self):
        """Every ghost we are tracking: both of them, all the time."""
        return [self.ghost, self.ghost2]

    def _advance(self, cell, t0):
        """Commit to driving into `cell` on the current heading."""
        self.cell = cell
        self._replan(t0)

    def _run_length(self):
        """Distance (m) from the current node to the next planned stop."""
        if self.action != "F":
            return 0.0
        n = 0
        for a, b in zip(self.path, self.path[1:]):
            if (a >> 2) == (b >> 2):
                return n * PITCH
            n += 1
        return math.inf if self.action == "X" else n * PITCH + PITCH

    def _correct_pose(self, s):
        """Pull the dead-reckoned pose toward what the ToF sensors see."""
        h = self.heading
        ux, uy = math.cos(HEADING_ANGLE[h]), math.sin(HEADING_ANGLE[h])
        cx, cy = self._chassis()
        cell = self.maze.cell_for_xy(cx, cy)
        ccx, ccy = self.maze.cell_center(cell)
        along = (cx - ccx) * ux + (cy - ccy) * uy
        lat = -(cx - ccx) * uy + (cy - ccy) * ux        # + = left of the line
        if abs(wrap(self.th - HEADING_ANGLE[h])) > 0.15:
            return
        self._frame = getattr(self, "_frame", 0) + 1
        if self._frame % 3 == 0 and abs(wrap(self.th - HEADING_ANGLE[h])) < 0.12:
            self._match_along(s)
        # Lateral: side walls of the cell the chassis centre is in.
        if abs(along) < 0.07:
            meas = None
            left_h, right_h = (h + 1) % 4, (h + 3) % 4
            lw = self._wall(cell, left_h) and 0.03 < s["fl"] < 0.17
            rw = self._wall(cell, right_h) and 0.03 < s["fr"] < 0.17
            if lw and rw:
                meas = 0.5 * (s["fr"] - s["fl"])
            elif lw:
                meas = SIDE_CLEAR - s["fl"]
            elif rw:
                meas = s["fr"] - SIDE_CLEAR
            if meas is not None:
                err = meas - lat
                k = 0.08
                self.x += -uy * err * k
                self.y += ux * err * k
        # Heading: the two forward rays hit the same flat wall ahead, so
        # their difference is the angle to it.
        ahead = self._ray_ahead(cell, h, along)
        if (ahead is not None and ahead < 0.6 and s["sl"] < 0.6
                and s["sr"] < 0.6 and abs(s["sl"] - s["sr"]) < 0.03):
            e = math.atan2(s["sl"] - s["sr"], FRONT_BASELINE)
            th_meas = HEADING_ANGLE[h] + e
            self.th = wrap(self.th + 0.1 * wrap(th_meas - self.th))
        # Along-track: the first wall ahead, when it is close and square on.
        if abs(lat) < 0.03 and abs(s["sl"] - s["sr"]) < 0.02 and s["sl"] < 1.3:
            expected = ahead
            if expected is not None and expected < 1.4:
                meas = 0.5 * (s["sl"] + s["sr"]) - FRONT_OFFSET
                err = expected - meas          # + means we are further along
                if abs(err) < 0.16:
                    k = 0.08
                    self.x += ux * err * k
                    self.y += uy * err * k

    def _ray(self, x, y, h):
        """Map distance from (x, y) along axis heading h to the first wall
        line, capped at TOF_MAX. Walls are SIDE_FACE thick on each side, so a
        ray passing within that of a cell corner can clip the neighbouring
        wall's end. Exit gaps let the ray out of the maze."""
        n, P = self.maze.grid, PITCH
        vw, hw = self.maze.v_walls, self.maze.h_walls
        if h in (0, 2):
            rows = self._lanes(n - 1 - y / P, n)
            if not rows:
                return TOF_MAX
            step = 1 if h == 0 else -1
            c = math.floor(x / P + 0.5) + (1 if h == 0 else 0)
            while 0 <= c <= n:
                d = ((c - 0.5) * P - x) * step
                if d > TOF_MAX:
                    return TOF_MAX
                if d > 0 and any(vw[c][r] == "#" for r in rows):
                    return d
                c += step
        else:
            cols = self._lanes(x / P, n)
            if not cols:
                return TOF_MAX
            step = -1 if h == 1 else 1
            r = math.floor(n - 1 - y / P + 0.5) + (0 if h == 1 else 1)
            while 0 <= r <= n:
                d = ((n - 1 - r + 0.5) * P - y) * -step
                if d > TOF_MAX:
                    return TOF_MAX
                if d > 0 and any(hw[r][c] == "#" for c in cols):
                    return d
                r += step
        return TOF_MAX

    @staticmethod
    def _lanes(u, n):
        """Cell indices whose wall segments a ray at grid coordinate u can hit
        (the lane it is in, plus a neighbour when within a wall's half
        thickness of the lane edge)."""
        i = int(round(u))
        lanes = [i]
        frac = u - i
        edge = SIDE_FACE / PITCH
        if frac > 0.5 - edge:
            lanes.append(i + 1)
        elif frac < -0.5 + edge:
            lanes.append(i - 1)
        return [k for k in lanes if 0 <= k < n]

    def _predict(self, cx, cy, h):
        """Expected (fl, fr, sl, sr) with the chassis centre at (cx, cy)."""
        ux, uy = math.cos(HEADING_ANGLE[h]), math.sin(HEADING_ANGLE[h])
        lx, ly = -uy, ux
        return (min(TOF_MAX, self._ray(cx, cy, (h + 1) % 4) - SIDE_FACE),
                min(TOF_MAX, self._ray(cx, cy, (h + 3) % 4) - SIDE_FACE),
                self._ray(cx + lx * FRONT_LAT, cy + ly * FRONT_LAT, h),
                self._ray(cx - lx * FRONT_LAT, cy - ly * FRONT_LAT, h))

    def _match_along(self, s):
        """Slide the pose along the corridor to where all four ToF readings
        fit the map best. Corrects drift that the front wall alone cannot
        (no wall ahead in range) using wall ends passing the side rays."""
        h = self.heading
        ux, uy = math.cos(HEADING_ANGLE[h]), math.sin(HEADING_ANGLE[h])
        cx, cy = self._chassis()
        meas = (s["fl"], s["fr"], s["sl"], s["sr"])
        costs = []
        for k in range(-MATCH_SPAN, MATCH_SPAN + 1):
            d = k * MATCH_STEP
            pred = self._predict(cx + ux * d, cy + uy * d, h)
            c = 0.0
            for p, m in zip(pred, meas):
                if p >= TOF_MAX - 0.05 and m >= TOF_MAX - 0.05:
                    continue
                c += min(abs(p - m), 0.08)
            costs.append(c)
        best = min(costs)
        here = costs[MATCH_SPAN]
        if best < 0.03 and here - best > 0.015:
            # Smallest move that explains the readings: a wall end passing a
            # side ray only says "not past here yet", so take the nearest
            # consistent position rather than an average.
            near = [k - MATCH_SPAN for k, c in enumerate(costs) if c <= best + 0.012]
            k_best = min(near, key=abs)
            shift = 0.7 * k_best * MATCH_STEP
            self.x += ux * shift
            self.y += uy * shift

    def _wall(self, cell, h):
        return not self.maze.can_move(cell, h) and not self._is_exit(cell, h)

    def _is_exit(self, cell, h):
        return any(c == cell and eh == h for c, eh in self.planner.exits)

    def _ray_ahead(self, cell, h, along):
        """Map distance from the chassis centre to the first wall ahead."""
        d = PITCH / 2 - along
        cur = cell
        for _ in range(12):
            if self._is_exit(cur, h):
                return None
            if not self.maze.can_move(cur, h):
                return d
            dr, dc = Maze.DELTA[h]
            cur = (cur[0] + dr, cur[1] + dc)
            d += PITCH
        return None

    # --------------------------------------------------------------- decide
    def decide(self, s):
        """One sensor frame in, one wheel command out.

        `s` is the decoded pacbot/sensors payload. Return (left, right) in
        rad/s.
        """
        dt = s["dt"]
        self.elapsed += dt

        yaw_rate = s["gyro"][2]      # rad/s about z
        # Ticks are cumulative counts, so speed is a DIFFERENCE, not a
        # reading. 360 counts per wheel revolution.
        tl, tr = s["ticks_l"], s["ticks_r"]
        if self.prev_ticks is None or dt <= 0:
            ds = 0.0
        else:
            dl, dr = tl - self.prev_ticks[0], tr - self.prev_ticks[1]
            ds = ENCODER_SCALE * WHEEL_RADIUS * ((dl + dr) / 2.0) * TICK_RAD
        self.prev_ticks = (tl, tr)
        speed = ds / dt if dt > 0 else 0.0
        self.speed = 0.5 * self.speed + 0.5 * speed
        self.yaw_f = 0.6 * self.yaw_f + 0.4 * yaw_rate

        # Dead reckoning: encoders for distance, gyro for heading.
        dth = yaw_rate * dt
        mid = self.th + dth / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = wrap(self.th + dth)

        self._track_ghosts()

        # Nothing to do until we know where the pellets are.
        if not self.have_pellets:
            return 0.0, 0.0

        if self.mode == "WAIT":
            self.mode = "STRAIGHT"
            self.cell = self.maze.spawn_cell
            self.heading = Maze.EAST
            self._replan(0.4)

        self.last_s = s
        if self.mode in ("STRAIGHT", "EXIT"):
            self._correct_pose(s)

        forward, turn = 0.0, 0.0
        if self.mode == "STRAIGHT":
            forward, turn = self._drive_straight(dt)
        if self.mode == "EXIT":
            forward, turn = self._drive_line(V_MAX, dt)
        if self.mode == "TURN":
            forward, turn = self._drive_turn(dt, yaw_rate)
        return forward, turn

    # ------------------------------------------------------------- motions
    def _drive_straight(self, dt):
        h = self.heading
        ux, uy = math.cos(HEADING_ANGLE[h]), math.sin(HEADING_ANGLE[h])
        ncx, ncy = self.maze.cell_center(self.cell)
        s_axle = (self.x - ncx) * ux + (self.y - ncy) * uy  # <0 before centre

        if self.action == "X":
            self.mode = "EXIT"
            return self._drive_line(V_MAX, dt)

        if self.action == "F" and s_axle >= 0.0:
            dr, dc = Maze.DELTA[h]
            nxt = (self.cell[0] + dr, self.cell[1] + dc)
            self._advance(nxt, PITCH / max(self.speed, 0.25))
            if self.action == "X":
                self.mode = "EXIT"
                return self._drive_line(V_MAX, dt)
            return self._drive_straight(dt)

        remaining = -s_axle + self._run_length()
        if self.action != "F":
            # Act on where we will be once this command lands.
            remaining -= STOP_BIAS + max(self.speed, 0.0) * LATENCY
            s = self.last_s
            at_wall = False
            if s is not None and not self.maze.can_move(self.cell, h):
                front = min(s["sl"], s["sr"])
                if front < 0.30:
                    sensed = front - FRONT_STOP - max(self.speed, 0.0) * LATENCY
                    if sensed < remaining:
                        remaining = sensed
                        at_wall = True
            if remaining < 0.002:
                if abs(self.speed) < 0.05:
                    if not at_wall and not self._turn_side_open():
                        # The side we are about to turn into still reads as
                        # wall: we stopped short. Trust the sensor, creep on.
                        self.x -= ux * 0.04
                        self.y -= uy * 0.04
                        return self._drive_line(0.08, dt)
                    return self._start_turn()
                self.v_cmd = 0.0
                return self._drive_line(0.0, dt)
        v_target = min(V_MAX, math.sqrt(2.0 * A_DEC * max(remaining, 0.0)))
        if self.action != "F":
            v_target = max(v_target, 0.05)
        return self._drive_line(v_target, dt)

    def _drive_line(self, v_target, dt):
        """Hold the corridor centre line on the current heading."""
        h = self.heading
        ux, uy = math.cos(HEADING_ANGLE[h]), math.sin(HEADING_ANGLE[h])
        lx, ly = self.maze.cell_center(self.cell)
        cx, cy = self._chassis()
        lat = -(cx - lx) * uy + (cy - ly) * ux       # + = left of the line
        lat += v_target * LATENCY * math.sin(wrap(self.th - HEADING_ANGLE[h]))
        th_des = HEADING_ANGLE[h] - max(-0.3, min(0.3, K_LAT * lat))
        err = wrap(th_des - (self.th + self.yaw_f * LATENCY))
        w = K_HEAD * err if abs(err) > 0.005 else 0.0
        w = max(-W_MAX, min(W_MAX, w))
        if v_target > self.v_cmd:
            self.v_cmd = min(v_target, self.v_cmd + A_ACC * dt)
        else:
            self.v_cmd = v_target
        self.w_cmd = w
        return self._wheels(self.v_cmd, w)

    def _turn_side_open(self):
        """Before an L/R pivot the ray on that side must see the opening."""
        s = self.last_s
        if s is None or self.action not in ("L", "R"):
            return True
        reading = s["fl"] if self.action == "L" else s["fr"]
        return reading > 0.16

    def _start_turn(self):
        delta = {"L": 1, "R": 3, "U": 2}[self.action]
        self.turn_target = (self.heading + delta) % 4
        if self.action == "L":
            self.turn_dir = 1
        elif self.action == "R":
            self.turn_dir = -1
        else:
            # A U-turn sweeps one side: swing toward whichever has more room.
            s = self.last_s
            self.turn_dir = 1 if (s is None or s["fl"] >= s["fr"]) else -1
        self.mode = "TURN"
        self.v_cmd = 0.0
        return 0.0, 0.0

    def _drive_turn(self, dt, yaw_rate):
        err = wrap(HEADING_ANGLE[self.turn_target] - self.th)
        if self.turn_dir > 0 and err < -2.0:
            err += 2.0 * math.pi
        elif self.turn_dir < 0 and err > 2.0:
            err -= 2.0 * math.pi
        # Rotation still to come once released: command latency plus spin-down.
        coast = self.yaw_f * LATENCY + self.yaw_f * abs(self.yaw_f) / (2.0 * TURN_ALPHA)
        err_pred = err - coast
        if abs(err) < TURN_DONE and abs(self.yaw_f) < 1.2:
            self.heading = self.turn_target
            self._replan(0.0, just_turned=True)
            if self.action == "F":
                dr, dc = Maze.DELTA[self.heading]
                nxt = (self.cell[0] + dr, self.cell[1] + dc)
                self.mode = "STRAIGHT"
                self._advance(nxt, PITCH / 0.3)
                if self.action == "X":
                    self.mode = "EXIT"
            elif self.action == "X":
                self.mode = "EXIT"
            else:
                self._start_turn()
            return 0.0, 0.0
        if abs(err_pred) < 0.03:
            turn = 0.0
        else:
            turn = min(WHEEL_LIMIT, max(TURN_MIN, TURN_GAIN * abs(err_pred)))
            turn = math.copysign(turn, err_pred)
        self.w_cmd = turn
        return wheels(0.0, turn)

    def _wheels(self, v, w):
        """(v m/s, w rad/s) -> (forward, turn) in wheel rad/s for wheels()."""
        forward = v / WHEEL_RADIUS
        turn = w * HALF_TRACK / WHEEL_RADIUS
        over = abs(forward) + abs(turn) - WHEEL_LIMIT
        if over > 0:
            forward = math.copysign(max(0.0, abs(forward) - over), forward)
        return wheels(forward, turn)


def main():
    # The official maze, embedded - no file to find, no path to pass.
    # The id printed here must match the one the simulator prints in its
    # banner ("maze official <id> (built in)"). If they ever differ, your
    # planner and the simulator are looking at different walls.
    maze = Maze.official()
    print(f"official maze {maze_3b.MAZE_ID}: {maze.grid}x{maze.grid}, "
          f"spawn row {maze.spawn_row}, exits {maze.exits}")

    controller = Controller(maze)
    client = _mqtt_client()

    def on_message(cli, userdata, msg):
        payload = json.loads(msg.payload.decode())
        if msg.topic == TOPIC_PELLETS:
            controller.on_pellets(payload)
            return
        if msg.topic == TOPIC_GHOST:
            controller.ghost = payload      # {"row": .., "col": .., "x": .., "y": ..}
            return
        if msg.topic == TOPIC_GHOST2:
            controller.ghost2 = payload     # same shape, ghost 2
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
    client.subscribe(TOPIC_GHOST2)
    client.subscribe(TOPIC_RESULT)
    client.subscribe(TOPIC_SENSORS)
    print("connected; waiting for the simulator")
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
