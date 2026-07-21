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
MY_POLICY_PATH = "/home/azureuser/go2_deploy/go2_flat_policy.pt"
MY_ENV_YAML    = "/home/azureuser/IsaacLab/logs/rsl_rl/unitree_go2_flat/2026-07-02_16-56-23/params/env.yaml"
MY_USD_PATH    = None   # dan usd_path Go2 tu env.yaml vao day de sua joint order
ENV_USD   = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
SPAWN_POS = [0.0, 0.0, 0.35]
SETTLE_STEPS = 100     # so buoc dung yen truoc khi cho dieu khien (~0.5s o 200Hz)
SPEED = 1.0
TURN  = 1.0
# ===============================================

first_step, reset_needed, warmup = True, False, 0
parser = argparse.ArgumentParser()
parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
args, _ = parser.parse_known_args()

# --- ban phim ---
keys = set()
def on_kbd(e, *a):
    if e.type in (KET.KEY_PRESS, KET.KEY_REPEAT): keys.add(e.input)
    elif e.type == KET.KEY_RELEASE: keys.discard(e.input)
    return True
_appwin = omni.appwindow.get_default_app_window()
_ip = carb.input.acquire_input_interface()
_sub = _ip.subscribe_to_keyboard_events(_appwin.get_keyboard(), on_kbd)


def on_physics_step(step_size, context):
    global first_step, reset_needed, warmup
    if first_step:
        go2.initialize()
        first_step = False
        warmup = 0
    elif reset_needed:
        reset_needed = False
        first_step = True
    elif warmup < SETTLE_STEPS:
        # pha settle: ghi robot o tu the default cho no dung vung
        go2.robot.set_dof_position_targets(positions=wp.from_torch(go2.default_pos))
        warmup += 1
        if warmup == SETTLE_STEPS:
            print(">>> Het settle — gio dieu khien bang W/S/A/D/Q/E")
    else:
        go2.forward(step_size, base_command)


assets = get_assets_root_path()
if assets is None:
    carb.log_error("Khong tim thay assets Isaac Sim")

ground = define_prim("/World/Ground", "Xform")
ground.GetReferences().AddReference(assets + ENV_USD)
define_prim("/World/PhysicsScene", "PhysicsScene")

RenderingManager.set_dt(8.0 / 200.0)
SimulationManager.set_physics_sim_device(args.device)
SimulationManager.set_physics_dt(1.0 / 200.0)

if USE_MY_POLICY:
    go2 = Go2Debug(
        prim_path="/World/Go2", position=SPAWN_POS,
        usd_path=MY_USD_PATH,
        policy_path=MY_POLICY_PATH, env_config_path=MY_ENV_YAML,
    )
    print(">>> POLICY CUA BAN:", MY_POLICY_PATH)
    print(">>> USD:", MY_USD_PATH if MY_USD_PATH else "(USD mac dinh)")
else:
    go2 = Go2Debug(prim_path="/World/Go2", position=SPAWN_POS)
    print(">>> POLICY NVIDIA (mac dinh)")
print(">>> Spawn -> settle dung yen ~0.5s -> W/S tien-lui, A/D ngang, Q/E xoay")

base_command = torch.zeros(3, device=args.device)
SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
omni.timeline.get_timeline_interface().play()
simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()
    if SimulationManager.is_simulating():
        vx = (SPEED if KB.W in keys else 0) - (SPEED if KB.S in keys else 0)
        vy = (SPEED if KB.A in keys else 0) - (SPEED if KB.D in keys else 0)
        wz = (TURN  if KB.Q in keys else 0) - (TURN  if KB.E in keys else 0)
        base_command = torch.tensor([float(vx), float(vy), float(wz)], device=args.device)
    else:
        reset_needed = True
simulation_app.close()