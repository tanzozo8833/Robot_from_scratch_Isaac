#!/usr/bin/env python3
"""
STEP 1 — Go2 + Isaac Sim 6.0.1 + ROS 2 Jazzy

Script standalone: nạp scene build_from_scratch.usd, chạy locomotion policy
(NVIDIA pre-trained) để Go2 bước đi theo /cmd_vel, gắn RTX LiDAR + camera RGB-D,
và publish đầy đủ topic ROS 2 cho slam_toolbox + Nav2.

Chạy:
    source /opt/ros/jazzy/setup.bash
    ~/isaacsim/python.sh ~/go2_deploy/go2_ros2.py

Xem README.md để biết cách điền CONFIG và thứ tự bring-up.
"""

# ============================================================================
# 1. CONFIG — ĐIỀN CÁC ĐƯỜNG DẪN NÀY TRƯỚC KHI CHẠY
# ============================================================================

# Scene (kết quả lệnh: find ~ -name "build_from_scratch.usd")
USD_SCENE_PATH = "/home/azureuser/build_from_scratch.usd"

# Prim path của Go2 trong scene. Để None -> script tự dò ArticulationRoot đầu tiên.
GO2_PRIM_PATH = None  # ví dụ: "/World/Go2"

# Policy: CẶP file .pt + env.yaml (phải cùng một lần train!)
#   NVIDIA pre-trained: find ~/isaacsim -iname "*.pt" -path "*olicy*"
#   Hoặc bản tự train:  ~/IsaacLab/logs/rsl_rl/unitree_go2_flat/<run>/exported/policy.pt
#                       ~/IsaacLab/logs/rsl_rl/unitree_go2_flat/<run>/params/env.yaml
POLICY_PT_PATH = "/home/azureuser/go2_deploy/policy.pt"
POLICY_ENV_YAML = "/home/azureuser/go2_deploy/env.yaml"

HEADLESS = False  # True nếu chạy không có DCV/display

# --- Frames (REP-105) ---
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
LIDAR_FRAME = "lidar_link"
CAMERA_FRAME = "front_cam"

# --- Topics ---
CMD_VEL_TOPIC = "/cmd_vel"       # Nav2 + teleop publish vào đây
ODOM_TOPIC = "/odom"
SCAN_TOPIC = "/scan"             # slam_toolbox ăn topic này
POINTS_TOPIC = "/points"
CAMERA_NS = "/go2/front_cam"     # -> /go2/front_cam/rgb, /depth, /camera_info

# --- Vị trí gắn cảm biến (mét, so với thân Go2) ---
LIDAR_XYZ = (0.0, 0.0, 0.12)     # trên lưng
CAMERA_XYZ = (0.30, 0.0, 0.02)   # trước đầu

# --- Giới hạn lệnh (khớp dải train của policy flat) ---
MAX_VX, MAX_VY, MAX_WZ = 0.8, 0.5, 1.0


# ============================================================================
# 2. KHỞI ĐỘNG ISAAC SIM  (SimulationApp PHẢI đứng trước mọi import khác)
# ============================================================================

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.rotations import quat_to_rot_matrix  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, open_stage  # noqa: E402
from pxr import Gf, UsdPhysics  # noqa: E402

# ROS 2 bridge phải bật TRƯỚC khi dựng OmniGraph có node ROS2
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402


# ============================================================================
# 3. TIỆN ÍCH
# ============================================================================


def find_articulation_root(stage):
    """Dò prim đầu tiên có ArticulationRootAPI (chính là con Go2)."""
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return None


def clamp(value, limit):
    return float(max(-limit, min(limit, value)))


# ============================================================================
# 4. NODE ROS 2: nhận /cmd_vel
# ============================================================================


class CmdVelListener(Node):
    """Node rclpy nhỏ chạy trong tiến trình Isaac, chỉ để nhận /cmd_vel.

    Isaac và ROS 2 Jazzy cùng Python 3.12 nên rclpy import được trực tiếp.
    """

    def __init__(self):
        super().__init__("go2_cmd_vel_listener")
        self.command = np.zeros(3, dtype=np.float32)  # [vx, vy, wz]
        self.create_subscription(Twist, CMD_VEL_TOPIC, self._on_cmd_vel, 10)

    def _on_cmd_vel(self, msg: Twist):
        # Giới hạn về dải mà policy được train -> tránh lệnh lạ làm Go2 ngã
        self.command[0] = clamp(msg.linear.x, MAX_VX)
        self.command[1] = clamp(msg.linear.y, MAX_VY)
        self.command[2] = clamp(msg.angular.z, MAX_WZ)


# ============================================================================
# 5. GẮN CẢM BIẾN
# ============================================================================


def create_lidar(parent_prim_path):
    """RTX LiDAR gắn lên lưng Go2 -> nguồn /scan cho slam_toolbox."""
    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/lidar",
        parent=parent_prim_path,
        config="Example_Rotary",  # LiDAR quay 360 độ
        translation=Gf.Vec3d(*LIDAR_XYZ),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    return sensor.GetPath().pathString


def create_camera(parent_prim_path):
    """Camera RGB-D gắn trước đầu Go2 -> dùng chính ở Step 2."""
    from isaacsim.sensors.camera import Camera

    camera = Camera(
        prim_path=f"{parent_prim_path}/front_cam",
        translation=np.array(CAMERA_XYZ),
        resolution=(640, 480),
    )
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()  # bật kênh depth
    return camera.prim_path


# ============================================================================
# 6. DỰNG OMNIGRAPH — cầu nối Isaac -> ROS 2
# ============================================================================


def build_clock_graph():
    """/clock — bắt buộc để mọi node ROS 2 chạy use_sim_time:=True."""
    og.Controller.edit(
        {"graph_path": "/ActionGraph_Clock", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [("PublishClock.inputs:topicName", "/clock")],
        },
    )


def build_odom_tf_graph(go2_prim):
    """/odom + TF odom->base_link + TF base_link->sensors.

    Đây là mắt xích mà Nav2 và slam_toolbox bắt buộc phải có.
    """
    og.Controller.edit(
        {"graph_path": "/ActionGraph_Odom", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PublishJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishRawTF.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishJoint.inputs:execIn"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishRawTF.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("Context.outputs:context", "PublishJoint.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishJoint.inputs:timeStamp"),
                # Compute -> Publish odometry
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                # Compute -> TF odom->base_link
                ("ComputeOdom.outputs:position", "PublishRawTF.inputs:translation"),
                ("ComputeOdom.outputs:orientation", "PublishRawTF.inputs:rotation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ComputeOdom.inputs:chassisPrim", [go2_prim]),
                ("PublishOdom.inputs:topicName", ODOM_TOPIC),
                ("PublishOdom.inputs:odomFrameId", ODOM_FRAME),
                ("PublishOdom.inputs:chassisFrameId", BASE_FRAME),
                # TF odom -> base_link
                ("PublishRawTF.inputs:parentFrameId", ODOM_FRAME),
                ("PublishRawTF.inputs:childFrameId", BASE_FRAME),
                # TF base_link -> các prim cảm biến (tĩnh)
                ("PublishTF.inputs:parentPrim", [go2_prim]),
                ("PublishTF.inputs:targetPrims", [go2_prim]),
                # /joint_states cho 12 khớp
                ("PublishJoint.inputs:topicName", "/joint_states"),
                ("PublishJoint.inputs:targetPrim", [go2_prim]),
            ],
        },
    )


def build_lidar_graph(lidar_prim):
    """RTX LiDAR -> /scan (LaserScan) + /points (PointCloud2)."""
    og.Controller.edit(
        {"graph_path": "/ActionGraph_Lidar", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RunOneFrame", "isaacsim.core.nodes.IsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("ScanHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("PointsHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "RunOneFrame.inputs:execIn"),
                ("RunOneFrame.outputs:step", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "ScanHelper.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PointsHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "ScanHelper.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "PointsHelper.inputs:renderProductPath"),
                ("Context.outputs:context", "ScanHelper.inputs:context"),
                ("Context.outputs:context", "PointsHelper.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [lidar_prim]),
                # /scan cho slam_toolbox
                ("ScanHelper.inputs:type", "laser_scan"),
                ("ScanHelper.inputs:topicName", SCAN_TOPIC),
                ("ScanHelper.inputs:frameId", LIDAR_FRAME),
                # /points (point cloud 3D)
                ("PointsHelper.inputs:type", "point_cloud"),
                ("PointsHelper.inputs:topicName", POINTS_TOPIC),
                ("PointsHelper.inputs:frameId", LIDAR_FRAME),
            ],
        },
    )


def build_camera_graph(camera_prim):
    """Camera -> /rgb + /depth + /camera_info (dùng chính ở Step 2)."""
    og.Controller.edit(
        {"graph_path": "/ActionGraph_Camera", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RunOneFrame", "isaacsim.core.nodes.IsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RgbHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("InfoHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "RunOneFrame.inputs:execIn"),
                ("RunOneFrame.outputs:step", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "RgbHelper.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "DepthHelper.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "InfoHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "RgbHelper.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "DepthHelper.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "InfoHelper.inputs:renderProductPath"),
                ("Context.outputs:context", "RgbHelper.inputs:context"),
                ("Context.outputs:context", "DepthHelper.inputs:context"),
                ("Context.outputs:context", "InfoHelper.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [camera_prim]),
                ("RgbHelper.inputs:type", "rgb"),
                ("RgbHelper.inputs:topicName", f"{CAMERA_NS}/rgb"),
                ("RgbHelper.inputs:frameId", CAMERA_FRAME),
                ("DepthHelper.inputs:type", "depth"),
                ("DepthHelper.inputs:topicName", f"{CAMERA_NS}/depth"),
                ("DepthHelper.inputs:frameId", CAMERA_FRAME),
                ("InfoHelper.inputs:type", "camera_info"),
                ("InfoHelper.inputs:topicName", f"{CAMERA_NS}/camera_info"),
                ("InfoHelper.inputs:frameId", CAMERA_FRAME),
            ],
        },
    )


# ============================================================================
# 7. POLICY CONTROLLER — /cmd_vel -> 48 obs -> 12 khớp
# ============================================================================

from isaacsim.robot.policy.examples.controllers.policy_controller import (  # noqa: E402
    PolicyController,
)


class Go2FlatPolicy(PolicyController):
    """Chạy policy locomotion flat của Go2.

    PolicyController tự lo 3 thứ hay khiến policy ngã khi deploy:
      1. Thứ tự 12 khớp -> lấy từ chính articulation (không hard-code).
      2. Observation scale -> đọc từ env.yaml.
      3. target_q = default_q + action_scale * action.
    Xem step1a-train-go2-policy.md để hiểu vì sao 3 điểm này quan trọng.
    """

    def __init__(self, prim_path, name="go2"):
        super().__init__(name=name, prim_path=prim_path)
        self.load_policy(POLICY_PT_PATH, POLICY_ENV_YAML)
        self._policy_counter = 0

    def _compute_observation(self, command):
        """Dựng vector 48 số ĐÚNG thứ tự lúc train (Isaac Lab velocity env).

        [0:3]   base linear velocity  (BODY frame)
        [3:6]   base angular velocity (BODY frame)
        [6:9]   projected gravity     (BODY frame)
        [9:12]  velocity command  <- chính là /cmd_vel
        [12:24] joint pos - default  (TƯƠNG ĐỐI)
        [24:36] joint vel
        [36:48] action bước trước

        LƯU Ý: get_linear_velocity() trả về vận tốc trong WORLD frame, nhưng policy
        được train với vận tốc trong BODY frame -> bắt buộc phải xoay về body frame
        bằng R_BI. Quên bước này = Go2 ngã ngay khi thân lệch khỏi hướng world.
        """
        lin_vel_w = self.robot.get_linear_velocity()
        ang_vel_w = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()  # quaternion thân trong world

        # R_BI = chuyển vị của R_IB -> đưa vector từ world về body frame
        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()

        lin_vel_b = np.matmul(R_BI, lin_vel_w)
        ang_vel_b = np.matmul(R_BI, ang_vel_w)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))  # trọng lực chiếu theo thân

        obs = np.zeros(48, dtype=np.float32)
        obs[0:3] = lin_vel_b
        obs[3:6] = ang_vel_b
        obs[6:9] = gravity_b
        obs[9:12] = command
        obs[12:24] = self.robot.get_joint_positions() - self.default_pos
        obs[24:36] = self.robot.get_joint_velocities()
        obs[36:48] = self._previous_action
        return obs

    def forward(self, dt, command):
        """Gọi mỗi physics step; chỉ inference mỗi `decimation` bước (-> 50 Hz)."""
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        # target_q = default_q + action_scale * action
        self.robot.set_joint_position_targets(self.default_pos + self.action * self._action_scale)
        self._policy_counter += 1


# ============================================================================
# 8. MAIN
# ============================================================================


def main():
    # --- Nạp scene ---
    carb.log_warn(f"[go2] Mở scene: {USD_SCENE_PATH}")
    open_stage(USD_SCENE_PATH)
    simulation_app.update()

    stage = get_current_stage()

    # --- Xác định prim Go2 ---
    go2_prim = GO2_PRIM_PATH or find_articulation_root(stage)
    if go2_prim is None:
        carb.log_error("[go2] Không tìm thấy ArticulationRoot. Hãy set GO2_PRIM_PATH thủ công.")
        simulation_app.close()
        return
    carb.log_warn(f"[go2] Prim robot: {go2_prim}")

    # --- Gắn cảm biến ---
    lidar_prim = create_lidar(go2_prim)
    camera_prim = create_camera(go2_prim)
    carb.log_warn(f"[go2] LiDAR: {lidar_prim} | Camera: {camera_prim}")

    # --- Dựng các graph ROS 2 ---
    build_clock_graph()
    build_odom_tf_graph(go2_prim)
    build_lidar_graph(lidar_prim)
    build_camera_graph(camera_prim)
    carb.log_warn("[go2] Đã dựng xong OmniGraph ROS 2")

    # --- World + policy ---
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=1.0 / 60.0)
    go2 = Go2FlatPolicy(prim_path=go2_prim)
    world.reset()
    go2.initialize()

    # In ra thứ tự khớp -> đối chiếu với step1a-train-go2-policy.md mục 5
    carb.log_warn(f"[go2] joint_names = {go2.robot.dof_names}")
    carb.log_warn(f"[go2] default_pos = {go2.default_pos}")

    # --- Node ROS 2 nhận /cmd_vel ---
    rclpy.init()
    cmd_node = CmdVelListener()
    carb.log_warn(f"[go2] Đang lắng nghe {CMD_VEL_TOPIC} — sẵn sàng nhận lệnh")

    # --- Vòng lặp chính ---
    try:
        while simulation_app.is_running():
            rclpy.spin_once(cmd_node, timeout_sec=0.0)
            go2.forward(world.get_physics_dt(), cmd_node.command)
            world.step(render=True)
    finally:
        cmd_node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
