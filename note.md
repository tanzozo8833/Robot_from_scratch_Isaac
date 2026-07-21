 azureuser@issas-sim-a10-u24-vm:~$ find ~/isaacsim -path "*robot.policy*" -name "*.py" 2>/dev/null | head -20
  /home/azureuser/isaacsim/standalone_examples/api/isaacsim.robot.policy.examples/spot_standalone.py
  /home/azureuser/isaacsim/standalone_examples/api/isaacsim.robot.policy.examples/anymal_standalone.py
  /home/azureuser/isaacsim/standalone_examples/api/isaacsim.robot.policy.examples/h1_standalone.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/utils/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/utils/actuator_network.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/quadruped/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/quadruped/quadruped_example.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/quadruped/quadruped_example_extension.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/utils.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/go2/go2_example_extension.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/go2/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/go2/go2_example.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/franka/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/franka/franka_example.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/franka/franka_example_extension.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/humanoid/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/humanoid/humanoid_example_extension.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/humanoid/humanoid_example.py
  azureuser@issas-sim-a10-u24-vm:~$ find ~/isaacsim -iname "*.pt" -path "*olicy*" 2>/dev/null | head
  /home/azureuser/isaacsim/exts/isaacsim.pip.newton/pip_prebundle/newton[sim]/newton-1.2.1-py3-none-any/newton/examples/assets/anymal_walking_policy.pt
  azureuser@issas-sim-a10-u24-vm:~$ find ~/isaacsim -iname "env.yaml" -path "*olicy*" 2>/dev/null | head
  azureuser@issas-sim-a10-u24-vm:~$ find ~ -name "build_from_scratch.usd" 2>/dev/null
  /home/azureuser/tan/build_from_scratch.usd




trong folder này t để file go2_teleop.py:
  """Go2 + policy trong warehouse: spawn -> settle dung yen -> dieu khien WASD."""
  from isaacsim import SimulationApp
  simulation_app = SimulationApp({"headless": False})

  import argparse, carb, carb.input
  import warp as wp
  import omni.timeline, omni.appwindow
  from isaacsim.core.deprecation_manager import import_module
  from isaacsim.core.experimental.utils.stage import define_prim
  from isaacsim.core.rendering_manager import RenderingManager
  from isaacsim.core.simulation_manager import SimulationManager
  from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
  from isaacsim.robot.policy.examples.robots import Go2FlatTerrainPolicy
  from isaacsim.storage.native import get_assets_root_path
  from carb.input import KeyboardInput as KB, KeyboardEventType as KET

  torch = import_module("torch")


  FLIP_GRAVITY = False   # dao dau truc Z cua gravity cho khop convention luc train

  class Go2Debug(Go2FlatTerrainPolicy):
      """Print observation + va loi lech dau gravity."""
      _n = 0
      def _compute_observation(self, command):
          obs = super()._compute_observation(command)
          Go2Debug._n += 1
          n = Go2Debug._n

          if n == 1:
              try:
                  print(">>> Thu tu 12 khop:", list(self.robot.dof_names))
              except Exception as e:
                  print(">>> khong lay duoc dof_names:", e)
              print(">>> default_pos:", [f"{x:+.2f}" for x in self.default_pos.cpu().numpy()])
              g = obs[6:9].detach().cpu().numpy()
              print(f">>> GRAVITY THO buoc 1 (chua va): [{g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f}]  (train mong doi Z ~ -1)")

          # === BAN VA ===
          if FLIP_GRAVITY:
              obs[6:9] = -obs[6:9]

          if n <= 3 or n % 50 == 0:   # in 3 buoc dau + moi ~1 giay
              o = obs.detach().cpu().numpy()
              f = lambda a: " ".join(f"{x:+.2f}" for x in a)
              tag = "SAU-VA" if FLIP_GRAVITY else "THO"
              print(f"--- obs #{n} ({tag}) ---")
              print(f" proj_gravity   [{f(o[6:9])}]   nen ~ [+0.00 +0.00 -1.00]")
              print(f" base_lin_vel   [{f(o[0:3])}]")
              print(f" base_ang_vel   [{f(o[3:6])}]")
              print(f" vel_command    [{f(o[9:12])}]")
              print(f" jointpos-def   [{f(o[12:24])}]")
          return obs


  # ================= CHỈNH Ở ĐÂY =================
  USE_MY_POLICY = False   # True = policy ban train | False = policy NVIDIA
  MY_POLICY_PATH = "/home/azureuser/go2_deploy/go2_flat_policy.
  ──── (103 lines hidden) ───────────────────────────────────────────────────────────────────
  /exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/interactive/utils.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/anymal.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/franka.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/go2.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/h1.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/spot.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/test_anymal.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/test_franka.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/test_go2.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/test_h1.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/test_interactive_utils.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/tests/test_spot.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/utils/__init__.py
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/utils/actuator_network.py
  azureuser@issas-sim-a10-u24-vm:~$ find ~/isaacsim -name "policy_controller.py" 2>/dev/null
  /home/azureuser/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/controllers/policy_controller.py




  azureuser@issas-sim-a10-u24-vm:~$ find ~/isaacsim -path "*ros2.bridge*" -name "*.ogn" | sed 's|.*/||' | sort
  azureuser@issas-sim-a10-u24-vm:~$

  azureuser@issas-sim-a10-u24-vm:~$ source /opt/ros/jazzy/setup.bash
  ~/isaacsim/python.sh -c "import rclpy; print('rclpy OK:', rclpy.__file__)"
  rclpy OK: /opt/ros/jazzy/lib/python3.12/site-packages/rclpy/__init__.py

  azureuser@issas-sim-a10-u24-vm:~$ find ~/isaacsim -iname "*.json" -path "*lidar*" 2>/dev/null | head -20
  /home/azureuser/isaacsim/exts/isaacsim.asset.importer.urdf/data/lidar_sensor_template/test_sensor_basic_lidar.json
  /home/azureuser/isaacsim/exts/isaacsim.asset.importer.urdf/data/lidar_sensor_template/lidar_template.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Velodyne/Velodyne_VLS128.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/SICK/SICK_multiScan165.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/SICK/SICK_multiScan136.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/SICK/SICK_tim781.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/SICK/SICK_picoScan150.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/SICK/SICK_microscan3_ABAZ90ZA1P01.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/HESAI/Hesai_XT32_SD10.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/NVIDIA/Debug_Rotary.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/NVIDIA/Simple_Example_Solid_State.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/ZVISION/ZVISION_ML30S.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/ZVISION/ZVISION_MLXS.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS1/OS1_REV6_128ch10hz2048res.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS1/OS1_REV6_128ch10hz1024res.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS1/OS1_REV7_128ch10hz2048res.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS1/OS1_REV7_128ch20hz512res.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS1/OS1_REV7_128ch20hz1024res.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS1/OS1_REV6_128ch20hz1024res.json
  /home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/Ouster/OS1/OS1_REV6_128ch20hz512res.json




  >>> Thu tu 12 khop: ['FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint', 'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint', 'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint']
  >>> default_pos: ['+0.10', '-0.10', '+0.10', '-0.10', '+0.80', '+0.80', '+1.00', '+1.00', '-1.50', '-1.50', '-1.50', '-1.50']




  azureuser@issas-sim-a10-u24-vm:~$ ls ~/isaacsim/exts | grep -iE "ros2|sensor"
  isaacsim.gui.sensors.icon
  isaacsim.ros2.bridge
  isaacsim.ros2.core
  isaacsim.ros2.examples
  isaacsim.ros2.nodes
  isaacsim.ros2.sim_control
  isaacsim.ros2.tf_viewer
  isaacsim.ros2.ui
  isaacsim.ros2.urdf
  isaacsim.sensors.camera.ui
  isaacsim.sensors.experimental.physics
  isaacsim.sensors.experimental.rtx
  isaacsim.sensors.physics.examples
  isaacsim.sensors.physics.nodes
  isaacsim.sensors.physics.ui
  isaacsim.sensors.rtx.nodes
  isaacsim.sensors.rtx.ui
  azureuser@issas-sim-a10-u24-vm:~$ ls ~/isaacsim/extsDeprecated | grep -iE "ros2|sensor"
  isaacsim.sensors.camera
  isaacsim.sensors.physics
  isaacsim.sensors.physx
  isaacsim.sensors.physx.examples
  isaacsim.sensors.physx.ui
  isaacsim.sensors.rtx