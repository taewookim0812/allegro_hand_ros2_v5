import carb
import omni
import omni.usd

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import get_default_context
from std_msgs.msg import Int32MultiArray

from omni.kit.scripting import BehaviorScript
from pxr import UsdShade, Gf, Sdf


# === Shared ROS context and executor setup ===
if not hasattr(rclpy, "_executor"):
    if not get_default_context().ok():
        rclpy.init()
    rclpy._executor = MultiThreadedExecutor()
    rclpy._nodes = set()

def register_node(node):
    if node not in rclpy._nodes:
        rclpy._executor.add_node(node)
        rclpy._nodes.add(node)

def spin_once():
    if hasattr(rclpy, "_executor"):
        rclpy._executor.spin_once(timeout_sec=0.0)


class SensorColorBehavior(BehaviorScript):
    def on_init(self):
        self._node = None
        self._sub = None
        self._latest_data = None
        carb.log_info(f"{type(self).__name__}.on_init() -> {self.prim_path}")

    def on_play(self):
        carb.log_info(f"{type(self).__name__}.on_play() -> {self.prim_path}")
        self._node = rclpy.create_node('sensor_color_node_left')
        self._sub = self._node.create_subscription(
            Int32MultiArray,
            'allegroHand_0/tactile_sensors',
            self._ros_callback,
            10
        )
        register_node(self._node)
        carb.log_info(f"{type(self).__name__}: ROS 2 subscriber registered.")

    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop() -> {self.prim_path}")
        if self._node:
            self._node.destroy_node()
        # No need to call rclpy.shutdown() since Isaac Sim runs all BehaviorScripts in a singleton Python process.
        # Calling shutdown() from one script may terminate ROS for other active scripts.

    def _ros_callback(self, msg: Int32MultiArray):
        self._latest_data = msg.data
        # print("Left: ", self._latest_data)

    def _compute_rgb(self, raw: float):
        value = min(raw * (255.0 / 500.0), 500.0)
        if value <= 64.0:
            return 0.0, value / 64.0, 1.0
        elif value <= 128.0:
            return 0.0, 1.0, 1.0 - ((value - 64.0) / 64.0)
        elif value <= 192.0:
            return (value - 128.0) / 64.0, 1.0, 0.0
        elif value <= 255.0:
            return 1.0, 1.0 - ((value - 192.0) / 64.0), 0.0
        else:
            return 1.0, 0.0, 0.0

    def on_update(self, current_time: float, delta_time: float):
        spin_once()

        data = self._latest_data
        if not data or len(data) < 4:
            return

        stage = omni.usd.get_context().get_stage()
        for idx, raw in enumerate(data[:4], start=1):
            r, g, b = self._compute_rgb(float(raw))
            prim_path = f"/World/allegro_hand_left/Looks/Fingertip{idx}/Shader"
            shader = UsdShade.Shader.Get(stage, Sdf.Path(prim_path))
            if not shader:
                carb.log_warn(f"Shader not found: {prim_path}")
                continue

            inp = shader.GetInput("diffuse_tint") or shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f)
            inp.Set(Gf.Vec3f(r, g, b))

        carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time}) -> {self.prim_path}")
