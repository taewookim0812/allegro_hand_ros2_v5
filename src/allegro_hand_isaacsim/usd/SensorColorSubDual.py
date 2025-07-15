import carb
import omni
import omni.usd

import rclpy
from rclpy.node import Node
from rclpy.utilities import get_default_context
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray

from omni.kit.scripting import BehaviorScript
from pxr import UsdShade, Gf, Sdf


class Sensorcolorsub(BehaviorScript):
    def on_init(self):
        self._node_left = None
        self._node_right = None
        self._latest_data_left = None
        self._latest_data_right = None
        self._executor = MultiThreadedExecutor()
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        ctx = get_default_context()
        if not ctx.ok():
            rclpy.init()
        ctx.on_shutdown(rclpy.shutdown)

        self._node_left = rclpy.create_node('sensor_color_node_left')
        self._node_right = rclpy.create_node('sensor_color_node_right')

        self._node_left.create_subscription(
            Int32MultiArray,
            '/allegroHand_0/tactile_sensors',
            self._ros_callback_left,
            10
        )
        self._node_right.create_subscription(
            Int32MultiArray,
            '/allegroHand_1/tactile_sensors',
            self._ros_callback_right,
            10
        )

        self._executor.add_node(self._node_left)
        self._executor.add_node(self._node_right)

        carb.log_info(f"{type(self).__name__}.on_play()->{self.prim_path}")

    def _ros_callback_left(self, msg: Int32MultiArray):
        self._latest_data_left = msg.data

    def _ros_callback_right(self, msg: Int32MultiArray):
        self._latest_data_right = msg.data

    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        if self._node_left:
            self._node_left.destroy_node()
        if self._node_right:
            self._node_right.destroy_node()
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")

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

    def _update_shader_colors(self, base_path: str, data: list):
        if not data or len(data) < 4:
            return

        stage = omni.usd.get_context().get_stage()

        for idx, raw in enumerate(data[:4], start=1):
            r, g, b = self._compute_rgb(float(raw))
            prim_path = f"{base_path}/Fingertip{idx}/Shader"
            shader = UsdShade.Shader.Get(stage, Sdf.Path(prim_path))

            if not shader:
                carb.log_warn(f"Shader not found: {prim_path}")
                continue

            inp = shader.GetInput("diffuse_tint") or shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f)
            inp.Set(Gf.Vec3f(r, g, b))

    def on_update(self, current_time: float, delta_time: float):
        if self._executor:
            self._executor.spin_once(timeout_sec=0.0)

        self._update_shader_colors("/World/allegro_hand_left/Looks", self._latest_data_left)
        self._update_shader_colors("/World/allegro_hand_right/Looks", self._latest_data_right)

        carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time})->{self.prim_path}")
