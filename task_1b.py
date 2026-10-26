"""PB Task 1B: left-hand wall follower with a PID lane controller.

Subscribes to the simulator's sensor topic and publishes one wheel velocity
command per sensors message.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b.py
"""
import json
import math
from collections import deque

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"   
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  


SWAP_SENSORS = True


GYRO_SIGN = 1.0 
WHEEL_SIGN = 1.0 


BASE_SPEED = 20.0
MAX_WHEEL = 20.0
TURN_MAX = 5.0 
TURN_MIN = 1.8
WHEEL_RADIUS = 0.016 


SENSOR_CAP = 0.6 
WALL_THR = 0.20 
STEER_MARGIN = 0.04
HALF_WIDTH0 = 0.09
FRONT_STOP = 0.075
FRONT_OPEN = 0.28
ADVANCE_DIST = 0.15
ADVANCE_STEP = 0.03     
ADVANCE_MAX = 0.24 
TOO_EARLY_FRONT = 0.15  
EXIT_DIST = 0.30


SETTLE_SAMPLES = 25
OPEN_CONFIRM = 12       
SEEN_CONFIRM = 50       


TURN_TIMEOUT = 6.0

KP = 4.0
KI = 0.3
KD = 0.25
KGYRO = 0.15
KH = 6.0
U_MAX = 3.0
I_MAX = 0.5
TURN_KP = 8.0
TURN_TOL = 0.05

FILTER_ALPHA = 0.25
WIDTH_ALPHA = 0.002


def _clean(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return SENSOR_CAP
    if math.isnan(x) or math.isinf(x):
        return SENSOR_CAP
    return min(max(x, 0.0), SENSOR_CAP)


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class _Sensor:

    def __init__(self):
        self.buf = deque(maxlen=3)
        self.val = None

    def update(self, raw):
        self.buf.append(_clean(raw))
        m = sorted(self.buf)[len(self.buf) // 2]
        if self.val is None:
            self.val = m
        else:
            self.val += FILTER_ALPHA * (m - self.val)
        return self.val

    def reset(self):
        self.buf.clear()
        self.val = None


class WallFollower:
    def __init__(self):
        self.state = "ENTER"
        self.hw = HALF_WIDTH0
        self.s_left = _Sensor()
        self.s_right = _Sensor()
        self.s_fa = _Sensor()
        self.s_fb = _Sensor()
        self.integral = 0.0
        self.prev_e = None
        self.timer = 0.0
        self.exit_dist = 0.0
        self.heading = 0.0
        self.heading_ref = 0.0
        self.heading_goal = 0.0
        self.turn_time = 0.0
        self.lw_cnt = 0
        self.lo_cnt = 0
        self.left_seen = False
        self.settle = 0
        self.advance_dist = ADVANCE_DIST
        self.gap_turn = False
        self.resume_advance = False

    def _reset_pid(self):
        self.integral = 0.0
        self.prev_e = None

    def _reset_sensing(self):
        for s in (self.s_left, self.s_right, self.s_fa, self.s_fb):
            s.reset()
        self.lw_cnt = 0
        self.lo_cnt = 0
        self.left_seen = False

    def _heading_term(self):
        """Steer back to the grid heading. > 0 means turn left."""
        return KH * _clamp(self.heading_ref - self.heading, -0.5, 0.5)

    def _steer(self, left_wall, right_wall, l, r, yaw_rate, dt):
        """PID correction u. u > 0 steers the bot to the left."""
        hw = self.hw
        if left_wall and right_wall:
            e = (l - r) / 2.0
        elif left_wall:
            e = l - hw
        elif right_wall:
            e = hw - r
        else:
            self._reset_pid()
            return _clamp(self._heading_term() - KGYRO * GYRO_SIGN * yaw_rate, -U_MAX, U_MAX)
        e /= hw

        self.integral = _clamp(self.integral + e * dt, -I_MAX, I_MAX)
        d = 0.0 if (self.prev_e is None or dt <= 0.0) else (e - self.prev_e) / dt
        self.prev_e = e
        u = (KP * e + KI * self.integral + KD * d + self._heading_term()
             - KGYRO * GYRO_SIGN * yaw_rate)
        return _clamp(u, -U_MAX, U_MAX)

    def _start_turn(self, angle):
        self.state = "TURN"
        self.heading_goal = self.heading_ref + angle
        self.turn_time = 0.0
        self.exit_dist = 0.0
        self._reset_pid()

    def _decide(self, l, r):
        if l >= WALL_THR:
            self._start_turn(+math.pi / 2.0)
        elif r >= WALL_THR:
            self._start_turn(-math.pi / 2.0)
        else:
            self._start_turn(math.pi)

    def step(self, fl, fr, sl, sr, yaw_rate, dt, accel_x=0.0):
        if SWAP_SENSORS:
            raw_l, raw_r, raw_fa, raw_fb = fl, fr, sl, sr
        else:
            raw_l, raw_r, raw_fa, raw_fb = sl, sr, fl, fr
        yaw_rate = float(yaw_rate) if yaw_rate is not None else 0.0
        dt = float(dt) if dt and dt > 0.0 else 0.002
        yaw = GYRO_SIGN * yaw_rate

        l = self.s_left.update(raw_l)
        r = self.s_right.update(raw_r)
        f = min(self.s_fa.update(raw_fa), self.s_fb.update(raw_fb))

        self.heading += yaw * dt

        if self.state == "DONE":
            return self._out(0.0, 0.0)

        if self.state == "TURN":
            self.turn_time += dt
            remaining = self.heading_goal - self.heading
            if abs(remaining) < TURN_TOL or self.turn_time > TURN_TIMEOUT:
                self.heading_ref = self.heading_goal
                self.state = "FOLLOW"
                self._reset_sensing()
                self._reset_pid()
                self.settle = SETTLE_SAMPLES
                if self.resume_advance:
                    self.resume_advance = False
                    self.state = "ADVANCE"
                    self.timer = ADVANCE_STEP / (BASE_SPEED * 0.8 * WHEEL_RADIUS)
                return self._out(BASE_SPEED * 0.6, BASE_SPEED * 0.6)
            mag = _clamp(TURN_KP * abs(remaining), TURN_MIN, TURN_MAX)
            w = mag if remaining > 0 else -mag
            return self._out(-w, w)

        if self.settle > 0:
            self.settle -= 1
            if self.settle == 0 and self.gap_turn:
                self.gap_turn = False
                if f < TOO_EARLY_FRONT and self.advance_dist < ADVANCE_MAX:
                    self.advance_dist = min(self.advance_dist + ADVANCE_STEP, ADVANCE_MAX)
                    self.resume_advance = True
                    self._start_turn(-math.pi / 2.0)
                    return self._out(0.0, 0.0)
            u = _clamp(self._heading_term() - KGYRO * GYRO_SIGN * yaw_rate, -U_MAX, U_MAX)
            return self._out(BASE_SPEED * 0.6 - u, BASE_SPEED * 0.6 + u)

        left_wall = l < WALL_THR
        right_wall = r < WALL_THR
        if left_wall:
            self.lw_cnt += 1
            self.lo_cnt = 0
        else:
            self.lo_cnt += 1
            self.lw_cnt = 0
        if self.lw_cnt >= SEEN_CONFIRM:
            self.left_seen = True
        left_open_confirmed = self.lo_cnt >= OPEN_CONFIRM
        front_wall = f < FRONT_STOP

        if l < 0.14 and r < 0.14:
            self.hw += WIDTH_ALPHA * (_clamp((l + r) / 2.0, 0.04, 0.13) - self.hw)

        left_steer = l < self.hw + STEER_MARGIN
        right_steer = r < self.hw + STEER_MARGIN

        speed = BASE_SPEED * _clamp((f - 0.05) / 0.12, 0.45, 1.0)

        if self.state == "ADVANCE":
            self.timer -= dt
            if front_wall:
                self._decide(l, r)
                return self._out(0.0, 0.0)
            if self.timer <= 0.0:
                if l >= WALL_THR and not (r >= WALL_THR and f > FRONT_OPEN):
                    self.gap_turn = True
                    self._start_turn(+math.pi / 2.0)
                    return self._out(0.0, 0.0)
                self.state = "FOLLOW"
                self.left_seen = False
                self.lw_cnt = 0
                return self._out(BASE_SPEED, BASE_SPEED)
            u = self._steer(False, right_steer, l, r, yaw_rate, dt)
            v = BASE_SPEED * 0.8
            return self._out(v - u, v + u)

        if (not left_wall) and (not right_wall) and f > FRONT_OPEN:
            self.exit_dist += BASE_SPEED * WHEEL_RADIUS * dt
            if self.exit_dist > EXIT_DIST:
                self.state = "DONE"
                return self._out(0.0, 0.0)
        else:
            self.exit_dist = 0.0

        if self.state == "ENTER":
            if front_wall:
                self.state = "FOLLOW"
                self._decide(l, r)
                return self._out(0.0, 0.0)
            if self.left_seen:
                self.state = "FOLLOW"
            else:
                u = self._steer(left_steer, right_steer, l, r, yaw_rate, dt)
                return self._out(speed - u, speed + u)

        if front_wall:
            self._decide(l, r)
            return self._out(0.0, 0.0)

        if self.left_seen and left_open_confirmed:
            self.state = "ADVANCE"
            self.timer = self.advance_dist / (BASE_SPEED * 0.8 * WHEEL_RADIUS)
            self._reset_pid()
            return self._out(BASE_SPEED * 0.8, BASE_SPEED * 0.8)

        # normal corridor driving
        u = self._steer(left_steer, right_steer, l, r, yaw_rate, dt)
        return self._out(speed - u, speed + u)

    @staticmethod
    def _out(left, right):
        left = _clamp(left, -MAX_WHEEL, MAX_WHEEL) * WHEEL_SIGN
        right = _clamp(right, -MAX_WHEEL, MAX_WHEEL) * WHEEL_SIGN
        return left, right


FOLLOWER = WallFollower()
_msg_count = 0


def _mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global _msg_count
    data = json.loads(msg.payload.decode())

    fl = data["fl"]
    fr = data["fr"]
    sl = data["sl"] 
    sr = data["sr"] 
    yaw_rate = data["gyro"][2]
    dt = data["dt"]

    _msg_count += 1
    if _msg_count % 50 == 0:
        print(f"fl={fl} fr={fr} sl={sl} sr={sr} "
              f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f} state={FOLLOWER.state}")

    accel_x = data["accel"][0]
    left_vel, right_vel = FOLLOWER.step(fl, fr, sl, sr, yaw_rate, dt, accel_x)

    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel), "right": float(right_vel),
    }))


def main():
    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass