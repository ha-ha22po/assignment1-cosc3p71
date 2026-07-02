#!/usr/bin/env python3
"""
JetAuto Teleop GUI
Requires: pip install websocket-client Pillow
"""

import io
import json
import math
import time
import threading
import os
import tkinter as tk
import urllib.request

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

try:
    import websocket
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False

# ── Robot config ──────────────────────────────────────────────────────────────
ROBOT_IP        = "100.97.124.22"
ROSBRIDGE_PORT  = 9090
WEB_VIDEO_PORT  = 8080
CAMERA_TOPIC    = "/depth_cam/rgb/image_raw"
CMD_VEL_TOPIC   = "/cmd_vel"
SERVO_TOPIC     = "/ros_robot_controller/bus_servo/set_position"

MAX_LINEAR  = 0.50   # m/s
MAX_ANGULAR = 1.50   # rad/s

SERVO_IDS      = [1, 2, 3, 4, 5, 10]
SERVO_DEFAULTS = {1: 500, 2: 750, 3: 0, 4: 375, 5: 500, 10: 500}
SERVO_NAMES    = {
    1:  "#1 Base Rotate",
    2:  "#2 Shoulder",
    3:  "#3 Elbow",
    4:  "#4 Wrist Pitch",
    5:  "#5 Gripper Rotation",
    10: "#6 Grip",
}

# ── Colors ────────────────────────────────────────────────────────────────────
BG      = "#1a1a2e"
PANEL   = "#16213e"
ACCENT  = "#0f3460"
ACCENT2 = "#1a4a80"
TEXT    = "#e0e0ff"
GREEN   = "#00cc66"
BLUE    = "#0066cc"
ORANGE  = "#cc6600"
RED     = "#cc2222"
PURPLE  = "#6060cc"
PURPLE2 = "#8080ff"


PROGRAM_FILE = "jetauto_final_program.json"

#PIL HSV hue range for blue/cyan blocks.
BLUE_H_MIN = 105
BLUE_H_MAX = 190
BLUE_S_MIN = 55
BLUE_V_MIN = 45

# Multi-color target options.
COLOR_RANGES = {
    "blue":  [((105, 190), 55, 45)],
    "green": [((55, 105), 45, 35)],
    "red":   [((0, 18), 55, 45), ((235, 255), 55, 45)],
}
VISION_SAMPLE_STEP = 4
VISION_MIN_PIXELS = 20

# Vision docking. the robot tries to make the blue block appear in the same
# camera position as it appeared when "Save Blue Target" is clicked
VISION_MAX_SECONDS = 12.0
VISION_STABLE_FRAMES = 6
VISION_X_TOL = 0.045
VISION_Y_TOL = 0.065
VISION_KX = 0.30
VISION_KY = 0.22
VISION_MAX_LX = 0.07
VISION_MAX_LY = 0.07

# if robot left/right, forward/backward moves are inverted
VISION_LX_SIGN = 1.0
VISION_LY_SIGN = -1.0

# to make arm move slowly so it does not push the block away while picking it up
PRE_PICKUP_DURATION = 2.0
PICKUP_OPEN_DURATION = 2.4
GRIP_CLOSE_DURATION = 1.2
LIFT_DURATION = 1.6


# ── Rosbridge client ──────────────────────────────────────────────────────────
class RosbridgeClient:
    def __init__(self, ip, port):
        self.ip, self.port = ip, port
        self.ws = None
        self.connected = False
        self._lock = threading.Lock()

    def connect(self):
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect(f"ws://{self.ip}:{self.port}", timeout=5)
            self.connected = True
            self._send({"op": "advertise", "topic": CMD_VEL_TOPIC,
                        "type": "geometry_msgs/Twist"})
            self._send({"op": "advertise", "topic": SERVO_TOPIC,
                        "type": "ros_robot_controller_msgs/ServosPosition"})
        except Exception as e:
            self.connected = False
            print(f"[rosbridge] connection failed: {e}")

    def _send(self, data):
        if not self.connected or self.ws is None:
            return
        with self._lock:
            try:
                self.ws.send(json.dumps(data))
            except Exception as e:
                self.connected = False
                print(f"[rosbridge] send error: {e}")

    def cmd_vel(self, lx, ly, az):
        self._send({"op": "publish", "topic": CMD_VEL_TOPIC,
                    "msg": {"linear":  {"x": float(lx), "y": float(ly), "z": 0.0},
                            "angular": {"x": 0.0,        "y": 0.0,       "z": float(az)}}})

    def set_servo(self, positions: dict, duration=0.1):
        self._send({"op": "publish", "topic": SERVO_TOPIC,
                    "msg": {"duration": float(duration),
                            "position": [{"id": k, "position": v}
                                         for k, v in positions.items()]}})

    def disconnect(self):
        self.connected = False
        try:
            if self.ws:
                self.ws.close()
        except Exception:
            pass


# ── Virtual joystick widget ───────────────────────────────────────────────────
class Joystick(tk.Canvas):
    def __init__(self, parent, size=200, callback=None, **kw):
        super().__init__(parent, width=size, height=size, **kw)
        self.size     = size
        self.center   = size // 2
        self.radius   = size // 2 - 12
        self.thumb_r  = 22
        self.callback = callback
        self.vx = self.vy = 0.0
        self._dragging = False
        self._draw()
        self.bind("<ButtonPress-1>",   self._press)
        self.bind("<B1-Motion>",       self._drag)
        self.bind("<ButtonRelease-1>", self._release)

    def _draw(self, tx=None, ty=None):
        self.delete("all")
        cx = cy = self.center
        r = self.radius
        # Background ring
        self.create_oval(cx - r, cy - r, cx + r, cy + r,
                         fill="#0d0d1e", outline="#3a3a7e", width=2)
        # Crosshair lines
        self.create_line(cx - r, cy, cx + r, cy, fill="#2a2a5e", width=1, dash=(4, 4))
        self.create_line(cx, cy - r, cx, cy + r, fill="#2a2a5e", width=1, dash=(4, 4))
        # Direction labels
        for txt, px, py in [("FWD", cx, cy - r + 10), ("BWD", cx, cy + r - 10),
                              ("◄",  cx - r + 10, cy), ("►",  cx + r - 10, cy)]:
            self.create_text(px, py, text=txt, fill="#5555aa", font=("Helvetica", 8))
        # Thumb
        tx = tx if tx is not None else cx
        ty = ty if ty is not None else cy
        self.create_oval(tx - self.thumb_r, ty - self.thumb_r,
                         tx + self.thumb_r, ty + self.thumb_r,
                         fill=PURPLE, outline=PURPLE2, width=2)
        # Value display
        if self.vx != 0 or self.vy != 0:
            self.create_text(cx, cy + r + 14, fill=TEXT,
                             text=f"x:{self.vy:+.2f}  y:{self.vx:+.2f}",
                             font=("Helvetica", 7))

    def _clamp(self, x, y):
        cx = cy = self.center
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        if d > self.radius:
            s = self.radius / d
            x, y = cx + dx * s, cy + dy * s
        return x, y

    def _press(self, e):
        self._dragging = True
        self._update(e.x, e.y)

    def _drag(self, e):
        if self._dragging:
            self._update(e.x, e.y)

    def _release(self, e):
        self._dragging = False
        self.vx = self.vy = 0.0
        self._draw()
        if self.callback:
            self.callback(0.0, 0.0)

    def _update(self, x, y):
        tx, ty = self._clamp(x, y)
        cx = cy = self.center
        self.vx = (tx - cx) / self.radius          # lateral  → linear_y
        self.vy = -(ty - cy) / self.radius          # vertical → linear_x
        self._draw(tx, ty)
        if self.callback:
            self.callback(self.vx, self.vy)


# ── Main application ──────────────────────────────────────────────────────────
class TeleopApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("JetAuto Teleop")
        self.root.configure(bg=BG)
        self.root.geometry("1050x720")

        self.client   = RosbridgeClient(ROBOT_IP, ROSBRIDGE_PORT)
        self.lx = self.ly = self.az = 0.0
        self.servo_vals = dict(SERVO_DEFAULTS)

        # Camera/vision state.
        self.latest_camera_frame = None
        self.vision_target = None
        self.target_color = tk.StringVar(value="blue")

        # Saved path points.
        # drive_points stores rough base paths. vision corrects the pickup endpoint.
        self.drive_points = {
            "to_pickup": None,
            "to_drop": None,
        }

        # separate arm/gripper poses.
        self.arm_points = {
            "view_pose": None,      # safe pose for driving/camera view
            "pre_pickup": None,     # optional high pose above block before lowering
            "pickup_open": None,    # arm above block, gripper open
            "grip_closed": None,    # same place, gripper closed around block
            "lifted": None,         # block lifted
            "drop_pose": None,      # arm over drop zone while holding
            "drop_open": None,      # release pose, gripper open
            
            "pickup": None,
            "drop": None,
        }
        self._recording_drive = None
        self._record_start_time = None
        self._record_samples = []   

        self._running = True

        self._build_ui()
        self._load_program()
        self._connect_async()
        if PIL_AVAILABLE:
            threading.Thread(target=self._camera_thread, daemon=True).start()
        self._publish_loop()
        self.root.protocol("WM_DELETE_WINDOW", self._quit)

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Title bar ─────────────────────────────────────────────────────────
        bar = tk.Frame(self.root, bg=ACCENT, pady=6)
        bar.pack(fill=tk.X)
        tk.Label(bar, text="  JetAuto Teleop Controller",
                 font=("Helvetica", 13, "bold"), fg=TEXT, bg=ACCENT).pack(side=tk.LEFT)
        self.status_lbl = tk.Label(bar, text="● Connecting…",
                                    font=("Helvetica", 10), fg="#ffaa00", bg=ACCENT)
        self.status_lbl.pack(side=tk.RIGHT, padx=12)

        # ── Content row ───────────────────────────────────────────────────────
        content = tk.Frame(self.root, bg=BG)
        content.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        left  = tk.Frame(content, bg=BG)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Scrollable right panel
        right_outer = tk.Frame(content, bg=BG)
        right_outer.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))

        right_canvas = tk.Canvas(right_outer, bg=BG, highlightthickness=0, width=330)
        right_scroll = tk.Scrollbar(right_outer, orient=tk.VERTICAL, command=right_canvas.yview)
        right_canvas.configure(yscrollcommand=right_scroll.set)

        right_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        right_canvas.pack(side=tk.LEFT, fill=tk.Y, expand=False)

        right = tk.LabelFrame(right_canvas, text=" Servo Control ",
                              font=("Helvetica", 10, "bold"), fg=TEXT, bg=PANEL,
                              bd=2, relief=tk.GROOVE)
        right_window = right_canvas.create_window((0, 0), window=right, anchor="nw")

        def _update_right_scrollregion(event=None):
            right_canvas.configure(scrollregion=right_canvas.bbox("all"))
            right_canvas.itemconfig(right_window, width=right_canvas.winfo_width())

        right.bind("<Configure>", _update_right_scrollregion)
        right_canvas.bind("<Configure>", _update_right_scrollregion)

        def _mousewheel(event):
            right_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        right_canvas.bind_all("<MouseWheel>", _mousewheel)

        # ── Camera ────────────────────────────────────────────────────────────
        cam_f = tk.LabelFrame(left, text=" Live Camera ",
                               font=("Helvetica", 10, "bold"), fg=TEXT, bg=PANEL,
                               bd=2, relief=tk.GROOVE)
        cam_f.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        self.cam_lbl = tk.Label(cam_f, bg="#080814", width=560, height=315)
        self.cam_lbl.pack(padx=4, pady=4)
        if not PIL_AVAILABLE:
            tk.Label(cam_f, text="⚠  pip install Pillow  to enable camera",
                     fg="#ff6666", bg="#080814", font=("Helvetica", 10)).pack()

        # ── Controls row ──────────────────────────────────────────────────────
        ctrl = tk.Frame(left, bg=BG)
        ctrl.pack(fill=tk.X)

        # Joystick
        joy_f = tk.LabelFrame(ctrl, text=" Drive (Drag) ",
                               font=("Helvetica", 9, "bold"), fg=TEXT, bg=PANEL,
                               bd=2, relief=tk.GROOVE)
        joy_f.pack(side=tk.LEFT, padx=(0, 8))
        self.joystick = Joystick(joy_f, size=210, callback=self._joy_cb,
                                  bg=PANEL, highlightthickness=0)
        self.joystick.pack(padx=8, pady=8)
        tk.Label(joy_f, text="Drag for X/Y drive · WASD keys also work",
                 fg="#555588", bg=PANEL, font=("Helvetica", 7)).pack(pady=(0, 4))

        # Turn + Stop
        ts_f = tk.LabelFrame(ctrl, text=" Turn / Stop ",
                              font=("Helvetica", 9, "bold"), fg=TEXT, bg=PANEL,
                              bd=2, relief=tk.GROOVE)
        ts_f.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        tk.Frame(ts_f, bg=PANEL, height=12).pack()

        turn_row = tk.Frame(ts_f, bg=PANEL)
        turn_row.pack(padx=10)

        bs = dict(font=("Helvetica", 22, "bold"), width=3, height=2,
                  bg=ACCENT, fg=TEXT, activebackground=ACCENT2,
                  activeforeground=TEXT, relief=tk.RAISED, bd=3, cursor="hand2")

        self.left_btn = tk.Button(turn_row, text="◄", **bs)
        self.left_btn.pack(side=tk.LEFT, padx=4)
        self.left_btn.bind("<ButtonPress-1>",   lambda e: self._set_az(MAX_ANGULAR))
        self.left_btn.bind("<ButtonRelease-1>", lambda e: self._set_az(0.0))

        self.right_btn = tk.Button(turn_row, text="►", **bs)
        self.right_btn.pack(side=tk.LEFT, padx=4)
        self.right_btn.bind("<ButtonPress-1>",   lambda e: self._set_az(-MAX_ANGULAR))
        self.right_btn.bind("<ButtonRelease-1>", lambda e: self._set_az(0.0))

        tk.Frame(ts_f, bg=PANEL, height=8).pack()
        tk.Button(ts_f, text="■  STOP", font=("Helvetica", 11, "bold"),
                  width=12, height=2, bg=RED, fg="white",
                  activebackground="#ee3333", cursor="hand2",
                  command=self._stop_all).pack(padx=10, pady=6)

        # Key hints
        hints = tk.Frame(ts_f, bg=PANEL)
        hints.pack(padx=6, pady=(0, 8))
        for txt, col in [("W/S: fwd/bwd", "#aaaaee"),
                          ("A/D: strafe",  "#aaaaee"),
                          ("Q/E: turn",    "#aaaaee"),
                          ("Space: stop",  "#ffaaaa")]:
            tk.Label(hints, text=txt, fg=col, bg=PANEL,
                     font=("Helvetica", 8)).pack(anchor="w")

        # Speed meters
        spd_f = tk.LabelFrame(ctrl, text=" Speed ",
                               font=("Helvetica", 9, "bold"), fg=TEXT, bg=PANEL,
                               bd=2, relief=tk.GROOVE)
        spd_f.pack(side=tk.LEFT, fill=tk.Y)
        self.spd_canvas = tk.Canvas(spd_f, width=90, height=210,
                                     bg=PANEL, highlightthickness=0)
        self.spd_canvas.pack(padx=6, pady=6)

        # ── Servo panel ───────────────────────────────────────────────────────
        self.servo_sliders = {}
        self.servo_val_lbl = {}

        for sid in SERVO_IDS:
            row = tk.Frame(right, bg=PANEL, pady=3)
            row.pack(fill=tk.X, padx=8)

            tk.Label(row, text=SERVO_NAMES[sid],
                     font=("Helvetica", 9, "bold"), fg=TEXT, bg=PANEL,
                     width=13, anchor="w").pack(side=tk.LEFT)

            vl = tk.Label(row, text=str(SERVO_DEFAULTS[sid]),
                           font=("Courier", 9), fg="#88ff88", bg=PANEL, width=4)
            vl.pack(side=tk.RIGHT)
            self.servo_val_lbl[sid] = vl

            # Center reset
            tk.Button(row, text="⊙", font=("Helvetica", 8), width=2,
                      bg=ACCENT, fg=TEXT, activebackground=ACCENT2, cursor="hand2",
                      command=lambda s=sid: self._center_servo(s)).pack(side=tk.RIGHT, padx=2)

            var = tk.IntVar(value=SERVO_DEFAULTS[sid])
            sl = tk.Scale(row, from_=0, to=1000, orient=tk.HORIZONTAL,
                           length=160, variable=var, showvalue=False,
                           command=lambda v, s=sid: self._servo_cb(s, int(v)),
                           bg=PANEL, fg=TEXT, troughcolor=ACCENT,
                           activebackground=PURPLE, highlightthickness=0)
            sl.set(SERVO_DEFAULTS[sid])
            sl.pack(side=tk.LEFT, padx=4)
            self.servo_sliders[sid] = sl

        tk.Frame(right, bg=PANEL, height=6).pack()

        tk.Button(right, text="Reset All Servos",
                  font=("Helvetica", 10, "bold"), bg="#006633", fg="white",
                  activebackground="#008844", cursor="hand2",
                  command=self._reset_servos).pack(padx=10, pady=(0, 10), fill=tk.X)

        # Pick/drop path recorder
        path_f = tk.LabelFrame(right, text=" Pick / Drop Program ",
                               font=("Helvetica", 9, "bold"), fg=TEXT, bg=PANEL,
                               bd=2, relief=tk.GROOVE)
        path_f.pack(fill=tk.X, padx=8, pady=(0, 10))

        tk.Label(path_f, text="1. Record drive to pickup\n2. Save pickup arm\n3. Record drive to drop\n4. Save drop arm",
                 fg="#aaaaff", bg=PANEL, font=("Helvetica", 8), justify=tk.LEFT).pack(anchor="w", padx=6, pady=4)

        tk.Button(path_f, text="Start Drive to Pickup", bg=ACCENT, fg=TEXT,
                  command=lambda: self._start_drive_record("to_pickup")).pack(fill=tk.X, padx=6, pady=2)
        tk.Button(path_f, text="Stop / Save Pickup Drive", bg=ACCENT, fg=TEXT,
                  command=self._stop_drive_record).pack(fill=tk.X, padx=6, pady=2)

        color_row = tk.Frame(path_f, bg=PANEL)
        color_row.pack(fill=tk.X, padx=6, pady=(4, 2))
        tk.Label(color_row, text="Target color:", fg=TEXT, bg=PANEL,
                 font=("Helvetica", 8, "bold")).pack(side=tk.LEFT)
        tk.OptionMenu(color_row, self.target_color, "blue", "red", "green").pack(side=tk.LEFT, padx=4)

        tk.Button(path_f, text="Save TARGET from camera", bg=PURPLE, fg="white",
                  font=("Helvetica", 8, "bold"),
                  command=self._save_blue_target).pack(fill=tk.X, padx=6, pady=(4, 2))
        tk.Button(path_f, text="VISION DOCK TO TARGET NOW", bg=PURPLE, fg="white",
                  font=("Helvetica", 8, "bold"),
                  command=self._vision_dock_async).pack(fill=tk.X, padx=6, pady=2)

        pose_f = tk.LabelFrame(path_f, text=" Arm / Gripper Poses ",
                               font=("Helvetica", 8, "bold"), fg=TEXT, bg=PANEL,
                               bd=1, relief=tk.GROOVE)
        pose_f.pack(fill=tk.X, padx=6, pady=(6, 4))

        tk.Button(pose_f, text="Save VIEW/CAMERA Pose", bg="#006633", fg="white",
                  command=lambda: self._save_arm_point("view_pose")).pack(fill=tk.X, padx=4, pady=1)
        tk.Button(pose_f, text="Move to VIEW/CAMERA Pose", bg=ACCENT, fg=TEXT,
                  command=lambda: threading.Thread(target=lambda: self._arm_to("view_pose", 1.0), daemon=True).start()).pack(fill=tk.X, padx=4, pady=1)

        tk.Button(pose_f, text="Save PRE-PICKUP Pose", bg="#006633", fg="white",
                  command=lambda: self._save_arm_point("pre_pickup")).pack(fill=tk.X, padx=4, pady=1)
        tk.Button(pose_f, text="Test PRE", bg=ACCENT, fg=TEXT,
                  command=lambda: threading.Thread(target=lambda: self._arm_to("pre_pickup", 1.2), daemon=True).start()).pack(fill=tk.X, padx=4, pady=1)

        tk.Button(pose_f, text="Save Pickup OPEN Pose", bg="#006633", fg="white",
                  command=lambda: self._save_arm_point("pickup_open")).pack(fill=tk.X, padx=4, pady=1)
        tk.Button(pose_f, text="Save Grip CLOSED Pose", bg="#006633", fg="white",
                  command=lambda: self._save_arm_point("grip_closed")).pack(fill=tk.X, padx=4, pady=1)
        tk.Button(pose_f, text="Save LIFTED Pose", bg="#006633", fg="white",
                  command=lambda: self._save_arm_point("lifted")).pack(fill=tk.X, padx=4, pady=1)
        tk.Button(pose_f, text="Save DROP Pose", bg="#006633", fg="white",
                  command=lambda: self._save_arm_point("drop_pose")).pack(fill=tk.X, padx=4, pady=1)
        tk.Button(pose_f, text="Save DROP OPEN Pose", bg="#006633", fg="white",
                  command=lambda: self._save_arm_point("drop_open")).pack(fill=tk.X, padx=4, pady=(1, 4))

        test_pose_f = tk.Frame(pose_f, bg=PANEL)
        test_pose_f.pack(fill=tk.X, padx=4, pady=(2, 4))
        tk.Button(test_pose_f, text="Test OPEN", bg=ACCENT, fg=TEXT,
                  command=lambda: threading.Thread(target=lambda: self._arm_to("pickup_open", 1.0), daemon=True).start()).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        tk.Button(test_pose_f, text="Test CLOSE", bg=ACCENT, fg=TEXT,
                  command=lambda: threading.Thread(target=lambda: self._arm_to("grip_closed", 1.0), daemon=True).start()).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        tk.Button(test_pose_f, text="Test LIFT", bg=ACCENT, fg=TEXT,
                  command=lambda: threading.Thread(target=lambda: self._arm_to("lifted", 1.0), daemon=True).start()).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        tk.Button(path_f, text="Start Drive to Drop", bg=ACCENT, fg=TEXT,
                  command=lambda: self._start_drive_record("to_drop")).pack(fill=tk.X, padx=6, pady=2)
        tk.Button(path_f, text="Stop / Save Drop Drive", bg=ACCENT, fg=TEXT,
                  command=self._stop_drive_record).pack(fill=tk.X, padx=6, pady=2)


        # Manual drive value entry.
        manual_f = tk.LabelFrame(path_f, text=" Manual Drive Values ",
                                 font=("Helvetica", 8, "bold"), fg=TEXT, bg=PANEL,
                                 bd=1, relief=tk.GROOVE)
        manual_f.pack(fill=tk.X, padx=6, pady=(6, 4))

        tk.Label(manual_f,
                 text="One step per line: lx, ly, az, seconds",
                 fg="#aaaaff", bg=PANEL, font=("Helvetica", 7)).pack(anchor="w", padx=4, pady=(2, 0))

        tk.Label(manual_f, text="Pickup path:", fg=TEXT, bg=PANEL,
                 font=("Helvetica", 7, "bold")).pack(anchor="w", padx=4)
        self.manual_pickup_txt = tk.Text(manual_f, height=3, width=30,
                                         bg="#080814", fg="#88ff88",
                                         insertbackground="#88ff88", font=("Courier", 8))
        self.manual_pickup_txt.insert("1.0", "0.25, 0, 0, 2.0\n0, 0, -0.8, 1.0\n0.12, 0, 0, 0.6")
        self.manual_pickup_txt.pack(fill=tk.X, padx=4, pady=(0, 2))

        tk.Button(manual_f, text="Save Manual Pickup Drive", bg=ACCENT, fg=TEXT,
                  command=lambda: self._save_manual_drive("to_pickup")).pack(fill=tk.X, padx=4, pady=1)

        tk.Label(manual_f, text="Drop path:", fg=TEXT, bg=PANEL,
                 font=("Helvetica", 7, "bold")).pack(anchor="w", padx=4, pady=(3, 0))
        self.manual_drop_txt = tk.Text(manual_f, height=3, width=30,
                                       bg="#080814", fg="#88ff88",
                                       insertbackground="#88ff88", font=("Courier", 8))
        self.manual_drop_txt.insert("1.0", "0, 0, -0.8, 2.0\n0.25, 0, 0, 3.0")
        self.manual_drop_txt.pack(fill=tk.X, padx=4, pady=(0, 2))

        tk.Button(manual_f, text="Save Manual Drop Drive", bg=ACCENT, fg=TEXT,
                  command=lambda: self._save_manual_drive("to_drop")).pack(fill=tk.X, padx=4, pady=(1, 4))

        tk.Button(path_f, text="RUN PICKUP ONLY TEST", bg=ORANGE, fg="white",
                  font=("Helvetica", 9, "bold"), command=self._run_pickup_only).pack(fill=tk.X, padx=6, pady=(6, 2))

        tk.Button(path_f, text="RUN FULL AUTO PICK/DROP", bg=ORANGE, fg="white",
                  font=("Helvetica", 9, "bold"), command=self._run_full_pick_drop).pack(fill=tk.X, padx=6, pady=(2, 2))

        tk.Button(path_f, text="Save Program / Paths", bg=ACCENT, fg=TEXT,
                  command=lambda: (self._save_program(), self._set_path_status(f"Saved {PROGRAM_FILE}"))).pack(fill=tk.X, padx=6, pady=(6, 2))
        tk.Button(path_f, text="Load Program / Paths", bg=ACCENT, fg=TEXT,
                  command=self._load_program).pack(fill=tk.X, padx=6, pady=(2, 2))

        self.path_status = tk.Label(path_f, text="No path saved yet", fg="#88ff88", bg=PANEL,
                                    font=("Helvetica", 8), wraplength=230, justify=tk.LEFT)
        self.path_status.pack(anchor="w", padx=6, pady=4)

        # ── Keyboard bindings ─────────────────────────────────────────────────
        self.root.bind("<KeyPress-w>",     lambda e: self._key_move( MAX_LINEAR, 0, 0))
        self.root.bind("<KeyPress-s>",     lambda e: self._key_move(-MAX_LINEAR, 0, 0))
        self.root.bind("<KeyPress-a>",     lambda e: self._key_move(0, -MAX_LINEAR, 0))
        self.root.bind("<KeyPress-d>",     lambda e: self._key_move(0,  MAX_LINEAR, 0))
        self.root.bind("<KeyPress-q>",     lambda e: self._key_move(0, 0,  MAX_ANGULAR))
        self.root.bind("<KeyPress-e>",     lambda e: self._key_move(0, 0, -MAX_ANGULAR))
        self.root.bind("<KeyRelease-w>",   lambda e: self._key_stop_fwd())
        self.root.bind("<KeyRelease-s>",   lambda e: self._key_stop_fwd())
        self.root.bind("<KeyRelease-a>",   lambda e: self._key_stop_lat())
        self.root.bind("<KeyRelease-d>",   lambda e: self._key_stop_lat())
        self.root.bind("<KeyRelease-q>",   lambda e: self._set_az(0.0))
        self.root.bind("<KeyRelease-e>",   lambda e: self._set_az(0.0))
        self.root.bind("<space>",          lambda e: self._stop_all())

    # ── Save / load program ───────────────────────────────────────────────────
    def _save_program(self):
        data = {
            "drive_points": self.drive_points,
            "arm_points": self.arm_points,
            "vision_target": self.vision_target,
            "target_color": self.target_color.get(),
        }
        try:
            with open(PROGRAM_FILE, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self._set_path_status(f"Could not save program file: {e}")

    def _load_program(self):
        if not os.path.exists(PROGRAM_FILE):
            return
        try:
            with open(PROGRAM_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.drive_points.update(data.get("drive_points", {}))
            self.arm_points.update(data.get("arm_points", {}))
            self.vision_target = data.get("vision_target")
            if "target_color" in data:
                self.target_color.set(data["target_color"])
            self._set_path_status(f"Loaded {PROGRAM_FILE}")
        except Exception as e:
            self._set_path_status(f"Could not load saved program: {e}")

    # ── Connection ────────────────────────────────────────────────────────────
    def _connect_async(self):
        def _run():
            self.client.connect()
            if self.client.connected:
                self.root.after(0, lambda: self.status_lbl.config(
                    text=f"● Connected  {ROBOT_IP}", fg=GREEN))
            else:
                self.root.after(0, lambda: self.status_lbl.config(
                    text="● Disconnected — retrying…", fg=RED))
                self.root.after(4000, self._connect_async)
        threading.Thread(target=_run, daemon=True).start()

    # ── Camera thread ─────────────────────────────────────────────────────────
    def _camera_thread(self):
        url = (f"http://{ROBOT_IP}:{WEB_VIDEO_PORT}/stream"
               f"?topic={CAMERA_TOPIC}&type=mjpeg&quality=65")
        while self._running:
            try:
                req = urllib.request.urlopen(url, timeout=8)
                buf = b""
                while self._running:
                    buf += req.read(8192)
                    while True:
                        s = buf.find(b"\xff\xd8")
                        e = buf.find(b"\xff\xd9", s + 2)
                        if s == -1 or e == -1:
                            break
                        jpg  = buf[s:e + 2]
                        buf  = buf[e + 2:]
                        try:
                            img   = Image.open(io.BytesIO(jpg)).resize((560, 315), Image.LANCZOS)
                            self.latest_camera_frame = img.copy()
                            photo = ImageTk.PhotoImage(img)
                            self.root.after(0, self._show_frame, photo)
                        except Exception:
                            pass
            except Exception as ex:
                if self._running:
                    time.sleep(2)

    def _show_frame(self, photo):
        self.cam_lbl.configure(image=photo)
        self.cam_lbl.image = photo

    # ── Control callbacks ─────────────────────────────────────────────────────
    def _joy_cb(self, vx, vy):
        self.ly = -vx * MAX_LINEAR  # lateral
        self.lx = vy * MAX_LINEAR   # forward

    def _set_az(self, val):
        self.az = val

    def _key_move(self, lx, ly, az):
        if lx != 0: self.lx = lx
        if ly != 0: self.ly = ly
        if az != 0: self.az = az

    def _key_stop_fwd(self): self.lx = 0.0
    def _key_stop_lat(self): self.ly = 0.0

    def _stop_all(self):
        self.lx = self.ly = self.az = 0.0
        self.joystick._draw()
        self.client.cmd_vel(0.0, 0.0, 0.0)

    def _servo_cb(self, sid, val):
        self.servo_vals[sid] = val
        self.servo_val_lbl[sid].config(text=str(val))
        self.client.set_servo({sid: val}, duration=0.08)

    def _center_servo(self, sid):
        self.servo_sliders[sid].set(500)

    def _reset_servos(self):
        for sid, val in SERVO_DEFAULTS.items():
            self.servo_sliders[sid].set(val)
        self.client.set_servo(SERVO_DEFAULTS, duration=1.0)


    # Pick/drop path recording
    def _set_path_status(self, text):
        print("[path]", text)
        if hasattr(self, "path_status"):
            self.root.after(0, lambda: self.path_status.config(text=text))

    def _start_drive_record(self, name):
        """Begin recording a driving move.

        Records the actual drive command stream (sampled in _publish_loop)
        instead of only the final joystick value, so playback matches the drive.
        """
        self._recording_drive = name
        self._record_start_time = time.time()
        self._record_samples = []
        self._set_path_status(f"Recording {name}. Drive the robot now, then click Stop/Save.")

    def _stop_drive_record(self):
        """Save the recorded drive as the captured command stream.

        The publish loop appended one (lx, ly, az) sample every ~0.1 s while
        you drove. We save those samples so playback re-issues the same
        commands for the same durations, instead of replaying a single
        instantaneous speed for the whole elapsed wall-clock time.
        """
        if self._recording_drive is None or self._record_start_time is None:
            self._set_path_status("No drive recording is active.")
            return

        dt = 0.1  # _publish_loop period (matches root.after(100, ...))
        samples = list(self._record_samples)

        def _moving(s):
            return abs(s[0]) > 0.001 or abs(s[1]) > 0.001 or abs(s[2]) > 0.001

        # Trim leading/trailing dead time (reaction lag before/after driving)
        # so the saved move only covers the part where the robot actually moved.
        start = 0
        while start < len(samples) and not _moving(samples[start]):
            start += 1
        end = len(samples)
        while end > start and not _moving(samples[end - 1]):
            end -= 1
        samples = samples[start:end]

        steps = [{"lx": lx, "ly": ly, "az": az, "seconds": dt}
                 for (lx, ly, az) in samples]
        moving = sum(1 for s in samples if _moving(s))

        saved_name = self._recording_drive
        self.drive_points[saved_name] = {"type": "recorded", "dt": dt, "steps": steps}
        self._save_program()
        self._recording_drive = None
        self._record_start_time = None
        self._record_samples = []
        self._stop_all()
        self._set_path_status(
            f"Saved {saved_name}: {len(steps)} samples, {moving} moving, "
            f"{len(steps) * dt:.2f} sec"
        )

    def _save_arm_point(self, name):
        """Save current arm/gripper slider values."""
        self.arm_points[name] = {str(k): int(v) for k, v in self.servo_vals.items()}
        self._save_program()
        self._set_path_status(f"Saved {name} pose: {self.arm_points[name]}")

    def _drive_for(self, move):
        """Drive using a saved move.

        move can be:
        - None
        - recorded dict: {"type": "recorded", "dt": ..., "steps": [...]}
          (played back as a continuous stream, with one stop at the end)
        - manual list:   [{"lx":..., "ly":..., "az":..., "seconds":...}, ...]
          (discrete steps, with a brief settle/stop between each)
        - legacy dict:   {"lx":..., "ly":..., "az":..., "seconds":...}
        """
        if move is None:
            return False

        # Recorded continuous drive to replay samples back to back so the robot
        # retraces the same path then stop once at the end.
        if isinstance(move, dict) and move.get("type") == "recorded":
            for step in move["steps"]:
                self.lx = float(step["lx"])
                self.ly = float(step["ly"])
                self.az = float(step["az"])
                time.sleep(float(step["seconds"]))
            self._stop_all()
            time.sleep(0.25)
            return True

        # Manual typed steps
        steps = move if isinstance(move, list) else [move]

        for step in steps:
            self.lx = float(step["lx"])
            self.ly = float(step["ly"])
            self.az = float(step["az"])
            time.sleep(float(step["seconds"]))
            self._stop_all()
            time.sleep(0.25)

        return True

    def _arm_to(self, point_name, duration=1.5):
        """Move the arm to a saved servo pose."""
        pose = self.arm_points.get(point_name)
        if pose is None:
            return False
        pose = {int(k): int(v) for k, v in pose.items()}
        self.client.set_servo(pose, duration=duration)
        for sid, val in pose.items():
            self.servo_vals[sid] = val
            if sid in self.servo_sliders:
                self.root.after(0, lambda s=sid, v=val: self.servo_sliders[s].set(v))
        time.sleep(duration + 0.3)
        return True

    def _grip_close(self):
        self.client.set_servo({10: 650}, duration=0.7)
        self.servo_vals[10] = 650
        time.sleep(0.9)

    def _grip_open(self):
        self.client.set_servo({10: 200}, duration=0.7)
        self.servo_vals[10] = 200
        time.sleep(0.9)

    def _parse_manual_drive_text(self, text):
        """Parse manual drive text.
        """
        steps = []
        for line_no, raw in enumerate(text.splitlines(), start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            # Allow either commas or spaces.
            line = line.replace(",", " ")
            parts = line.split()
            if len(parts) != 4:
                raise ValueError(
                    f"Line {line_no}: expected 4 values: lx, ly, az, seconds"
                )

            lx, ly, az, seconds = [float(p) for p in parts]
            if seconds <= 0:
                raise ValueError(f"Line {line_no}: seconds must be greater than 0")

            
            lx = max(-MAX_LINEAR, min(MAX_LINEAR, lx))
            ly = max(-MAX_LINEAR, min(MAX_LINEAR, ly))
            az = max(-MAX_ANGULAR, min(MAX_ANGULAR, az))

            steps.append({
                "lx": lx,
                "ly": ly,
                "az": az,
                "seconds": seconds,
            })

        if not steps:
            raise ValueError("No manual drive steps were entered.")

        return steps

    def _save_manual_drive(self, name):
        """Save manually entered drive steps for pickup or drop."""
        try:
            if name == "to_pickup":
                raw = self.manual_pickup_txt.get("1.0", tk.END)
                label = "pickup"
            elif name == "to_drop":
                raw = self.manual_drop_txt.get("1.0", tk.END)
                label = "drop"
            else:
                self._set_path_status(f"Unknown manual drive name: {name}")
                return

            steps = self._parse_manual_drive_text(raw)
            self.drive_points[name] = steps
            self._save_program()

            moving_steps = sum(
                1 for s in steps
                if abs(s["lx"]) > 0.001 or abs(s["ly"]) > 0.001 or abs(s["az"]) > 0.001
            )
            total_time = sum(s["seconds"] for s in steps)

            self._set_path_status(
                f"Saved manual {label} drive: {len(steps)} steps, "
                f"{moving_steps} moving, {total_time:.2f} sec"
            )

        except Exception as e:
            self._set_path_status(f"Manual drive error: {e}")


    # ── Blue block vision docking ─────────────────────────────────────────────
    def _detect_blue_block(self):
        """Detect the selected target color in the current camera frame.

        """
        img = self.latest_camera_frame
        if img is None:
            return None

        w, h = img.size
        hsv = img.convert("HSV")
        pix = hsv.load()

        color_name = self.target_color.get()
        ranges = COLOR_RANGES.get(color_name, COLOR_RANGES["blue"])

        def matches(hh, ss, vv):
            for (h_range, s_min, v_min) in ranges:
                h_min, h_max = h_range
                if h_min <= hh <= h_max and ss >= s_min and vv >= v_min:
                    return True
            return False

        # The block is on the floor so ignore the top of the image.
        y_start = int(h * 0.20)

        count = 0
        sx = 0
        sy = 0
        min_x, min_y = w, h
        max_x, max_y = 0, 0

        for y in range(y_start, h, VISION_SAMPLE_STEP):
            for x in range(0, w, VISION_SAMPLE_STEP):
                hh, ss, vv = pix[x, y]
                if matches(hh, ss, vv):
                    count += 1
                    sx += x
                    sy += y
                    if x < min_x: min_x = x
                    if y < min_y: min_y = y
                    if x > max_x: max_x = x
                    if y > max_y: max_y = y

        if count < VISION_MIN_PIXELS:
            return None

        cx = sx / count
        cy = sy / count
        return {
            "x_frac": cx / w,
            "y_frac": cy / h,
            "count": count,
            "bbox_w": max_x - min_x,
            "bbox_h": max_y - min_y,
            "color": color_name,
        }

    def _save_blue_target(self):
        """Save where the block appears when the robot is perfectly lined up.

        """
        det = self._detect_blue_block()
        if det is None:
            self._set_path_status("Target not saved: selected color block not detected in camera.")
            return

        self.vision_target = {
            "x_frac": det["x_frac"],
            "y_frac": det["y_frac"],
            "bbox_h": det["bbox_h"],
        }
        self._save_program()
        self._set_path_status(
            f"Saved {self.target_color.get()} target: x={det['x_frac']:.3f}, y={det['y_frac']:.3f}, bbox_h={det['bbox_h']:.0f}"
        )

    def _vision_dock_to_blue(self, max_seconds=VISION_MAX_SECONDS):
        """Move slowly until the blue block matches the saved target position."""
        if self.vision_target is None:
            self._set_path_status("Vision dock failed: click Save TARGET first.")
            return False

        start = time.time()
        stable = 0
        seen = False

        while self._running and time.time() - start < max_seconds:
            det = self._detect_blue_block()
            if det is None:
                self._stop_all()
                msg = "lost blue" if seen else "blue not visible"
                self._set_path_status(f"Vision dock waiting: {msg}. Rough path must stop with blue visible.")
                time.sleep(0.15)
                continue

            seen = True
            err_x = det["x_frac"] - float(self.vision_target["x_frac"])
            err_y = det["y_frac"] - float(self.vision_target["y_frac"])

            if abs(err_x) <= VISION_X_TOL and abs(err_y) <= VISION_Y_TOL:
                stable += 1
                self._stop_all()
                self._set_path_status(
                    f"Vision dock stable {stable}/{VISION_STABLE_FRAMES}: xerr={err_x:+.3f}, yerr={err_y:+.3f}"
                )
                if stable >= VISION_STABLE_FRAMES:
                    self._set_path_status("Vision dock done: target lined up.")
                    return True
                time.sleep(0.12)
                continue

            stable = 0

            
            
            ly = VISION_LY_SIGN * VISION_KX * err_x
            lx = VISION_LX_SIGN * (-VISION_KY * err_y)

            if abs(err_x) <= VISION_X_TOL:
                ly = 0.0
            if abs(err_y) <= VISION_Y_TOL:
                lx = 0.0

            lx = max(-VISION_MAX_LX, min(VISION_MAX_LX, lx))
            ly = max(-VISION_MAX_LY, min(VISION_MAX_LY, ly))

            self.lx = lx
            self.ly = ly
            self.az = 0.0
            self._set_path_status(
                f"Vision dock: xerr={err_x:+.3f}, yerr={err_y:+.3f}, lx={lx:+.2f}, ly={ly:+.2f}"
            )
            time.sleep(0.12)

        self._stop_all()
        self._set_path_status("Vision dock timed out. Move rough path closer or re-save target.")
        return False

    def _vision_dock_async(self):
        threading.Thread(target=lambda: self._vision_dock_to_blue(), daemon=True).start()


    def _run_pickup_only(self):
        """to test only pickup: rough path -> vision dock -> open pose -> close pose -> lifted pose."""
        missing = []
        if self.drive_points["to_pickup"] is None:
            missing.append("rough drive to pickup")
        if self.vision_target is None:
            missing.append("blue vision target")
        for name in ["pickup_open", "grip_closed", "lifted"]:
            if self.arm_points.get(name) is None:
                missing.append(name + " pose")
        if missing:
            self._set_path_status("Missing: " + ", ".join(missing))
            return

        def sequence():
            if self.arm_points.get("view_pose") is not None:
                self._set_path_status("Pickup test: moving to VIEW/CAMERA pose")
                self._arm_to("view_pose", duration=1.0)

            self._set_path_status("Pickup test: rough drive to block area")
            self._drive_for(self.drive_points["to_pickup"])

            self._set_path_status("Pickup test: vision docking")
            if not self._vision_dock_to_blue():
                self._set_path_status("Pickup test stopped: vision docking failed.")
                return

            if self.arm_points.get("pre_pickup") is not None:
                self._set_path_status("Pickup test: moving to PRE-PICKUP pose")
                self._arm_to("pre_pickup", duration=PRE_PICKUP_DURATION)

            self._set_path_status("Pickup test: moving slowly to pickup OPEN pose")
            self._arm_to("pickup_open", duration=PICKUP_OPEN_DURATION)

            self._set_path_status("Pickup test: closing grip")
            self._arm_to("grip_closed", duration=GRIP_CLOSE_DURATION)

            self._set_path_status("Pickup test: lifting")
            self._arm_to("lifted", duration=LIFT_DURATION)

            self._set_path_status("Pickup test done.")

        threading.Thread(target=sequence, daemon=True).start()

    def _run_full_pick_drop(self):
        """Run full autonomous pickup and drop offsequence:
        rough path -> vision dock -> pickup open -> grip closed -> lift ->
        drive to drop -> drop pose -> drop open -> lift.
        """
        missing = []
        if self.drive_points["to_pickup"] is None:
            missing.append("rough drive to pickup")
        if self.vision_target is None:
            missing.append("blue vision target")
        if self.drive_points["to_drop"] is None:
            missing.append("drive to drop")
        for name in ["pickup_open", "grip_closed", "lifted", "drop_pose", "drop_open"]:
            if self.arm_points.get(name) is None:
                missing.append(name + " pose")

        if missing:
            self._set_path_status("Missing: " + ", ".join(missing))
            return

        def sequence():
            if self.arm_points.get("view_pose") is not None:
                self._set_path_status("Moving to VIEW/CAMERA pose")
                self._arm_to("view_pose", duration=1.0)

            self._set_path_status("Running rough drive to block area")
            self._drive_for(self.drive_points["to_pickup"])

            self._set_path_status("Vision docking to selected target")
            if not self._vision_dock_to_blue():
                self._set_path_status("Stopped: vision docking failed.")
                return

            if self.arm_points.get("pre_pickup") is not None:
                self._set_path_status("Moving to PRE-PICKUP pose")
                self._arm_to("pre_pickup", duration=PRE_PICKUP_DURATION)

            self._set_path_status("Moving slowly to pickup OPEN pose")
            self._arm_to("pickup_open", duration=PICKUP_OPEN_DURATION)

            self._set_path_status("Closing grip with saved CLOSED pose")
            self._arm_to("grip_closed", duration=GRIP_CLOSE_DURATION)

            self._set_path_status("Lifting block")
            self._arm_to("lifted", duration=LIFT_DURATION)

            self._set_path_status("Driving to drop zone")
            self._drive_for(self.drive_points["to_drop"])

            self._set_path_status("Moving to drop pose")
            self._arm_to("drop_pose", duration=1.5)

            self._set_path_status("Opening gripper / releasing block")
            self._arm_to("drop_open", duration=1.0)

            self._set_path_status("Raising arm after drop")
            self._arm_to("lifted", duration=1.2)

            self._stop_all()
            self._set_path_status("Done")

        threading.Thread(target=sequence, daemon=True).start()

    # ── Speed meter ───────────────────────────────────────────────────────────
    def _draw_speed(self):
        c  = self.spd_canvas
        W, H = 90, 210
        c.delete("all")
        bars = [("Vx fwd", self.lx, MAX_LINEAR, GREEN),
                ("Vy lat", self.ly, MAX_LINEAR, BLUE),
                ("ω turn", self.az, MAX_ANGULAR, ORANGE)]
        for i, (label, val, mx, color) in enumerate(bars):
            y0 = 10 + i * 65
            c.create_text(W // 2, y0, text=label, fill="#9999cc",
                          font=("Helvetica", 8))
            bx, by = 10, y0 + 12
            bw, bh = W - 20, 22
            c.create_rectangle(bx, by, bx + bw, by + bh,
                                fill="#080814", outline="#2a2a5e")
            mid = bx + bw // 2
            c.create_line(mid, by, mid, by + bh, fill="#333366")
            fill = int(abs(val) / mx * (bw // 2))
            if val >= 0:
                c.create_rectangle(mid, by + 2, mid + fill, by + bh - 2, fill=color)
            else:
                c.create_rectangle(mid - fill, by + 2, mid, by + bh - 2, fill=color)
            c.create_text(W // 2, by + bh + 6, text=f"{val:+.2f}",
                          fill=color, font=("Courier", 8))

    # ── 10 Hz publish loop ────────────────────────────────────────────────────
    def _publish_loop(self):
        if self._running:
            self.client.cmd_vel(self.lx, self.ly, self.az)
            # Continuously capture the drive command while recording a path.
            if self._recording_drive is not None:
                self._record_samples.append((self.lx, self.ly, self.az))
            self._draw_speed()
            self.root.after(100, self._publish_loop)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def _quit(self):
        self._running = False
        self.client.cmd_vel(0.0, 0.0, 0.0)
        time.sleep(0.12)
        self.client.disconnect()
        self.root.destroy()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    missing = []
    if not WS_AVAILABLE:
        missing.append("websocket-client")
    if not PIL_AVAILABLE:
        missing.append("Pillow")
    if missing:
        print(f"\n  Install missing packages:  pip install {' '.join(missing)}\n")
        if not WS_AVAILABLE:
            print("  websocket-client is required to connect to the robot.")
            return

    root = tk.Tk()
    root.resizable(True, True)
    TeleopApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
