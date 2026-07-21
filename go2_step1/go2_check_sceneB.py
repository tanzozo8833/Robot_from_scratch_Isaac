"""STAGE B0 — Kiem tra huong B: attach policy NVIDIA vao con Go2 CO SAN trong scene.

Muc dich:
  1. Mo build_from_scratch.usd (warehouse + Go2 user da keo tha).
  2. Tu do ArticulationRoot -> tim con Go2 that su.
  3. Attach Go2FlatTerrainPolicy vao prim CO SAN (khong spawn them con thu 2).
  4. In dof_names + default_pos + physics engine + co PhysicsScene khong.
  5. Settle 100 buoc roi cho dieu khien WASD.

Chi dung API da CHUNG MINH chay duoc trong go2_teleop.py. Khong dung API doan.

Chay:
    ~/isaacsim/python.sh ~/go2_deploy/go2_check_sceneB.py

Neu Go2 DI DUOC -> huong B song, doc log ">>> Thu tu 12 khop" gui lai.
Neu Go2 NGA     -> joint order asset Unitree khac policy -> bao lai de xu ly.
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import carb
import carb.input
import omni.appwindow
import omni.timeline
import omni.usd
import warp as wp
from carb.input import KeyboardEventType as KET
from carb.input import KeyboardInput as KB
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots import Go2FlatTerrainPolicy
from pxr import UsdPhysics

torch = import_module("torch")


# ================= CHINH O DAY =================
USD_PATH = "/home/azureuser/tan/build_from_scratch.usd"
GO2_PRIM = None  # None = tu do ArticulationRoot. Neu biet chinh xac thi dien, vd "/World/go2"
SETTLE_STEPS = 100
SPEED = 1.0
TURN = 1.0
FLIP_GRAVITY = False
# ===============================================


class Go2Debug(Go2FlatTerrainPolicy):
    """In observation + dof order o buoc dau (giong go2_teleop.py)."""

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
            print(f">>> GRAVITY THO buoc 1: [{g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f}]  (train mong doi Z ~ -1)")
        if FLIP_GRAVITY:
            obs[6:9] = -obs[6:9]
        return obs


parser = argparse.ArgumentParser()
parser.add_argument("--device", choices=["cpu", "cuda"], default="cpu")
args, _ = parser.parse_known_args()

# --- ban phim (giong go2_teleop.py) ---
keys = set()


def on_kbd(e, *a):
    if e.type in (KET.KEY_PRESS, KET.KEY_REPEAT):
        keys.add(e.input)
    elif e.type == KET.KEY_RELEASE:
        keys.discard(e.input)
    return True


_appwin = omni.appwindow.get_default_app_window()
_ip = carb.input.acquire_input_interface()
_sub = _ip.subscribe_to_keyboard_events(_appwin.get_keyboard(), on_kbd)


# ============================================================================
# 1. MO SCENE + doi payload (warehouse, go2) tai xong tu asset server
# ============================================================================
print(f">>> Mo scene: {USD_PATH}")
ctx = omni.usd.get_context()
ctx.open_stage(USD_PATH)


def find_articulation_roots(stage):
    return [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]


# Payload tai tu https:// -> can vai chuc/ tram lan update moi xong. Doi toi khi thay articulation.
roots = []
for i in range(600):
    simulation_app.update()
    stage = ctx.get_stage()
    roots = find_articulation_roots(stage)
    if roots:
        print(f">>> Da nap xong sau {i} lan update")
        break

stage = ctx.get_stage()
print(">>> ArticulationRoot tim thay:", roots if roots else "(KHONG CO — payload chua nap? kiem tra mang/asset)")

# --- co PhysicsScene chua? thieu thi them, khong physics se khong chay ---
has_phys = any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse())
print(">>> Co PhysicsScene:", has_phys)
if not has_phys:
    print(">>> Chua co PhysicsScene -> tao /World/PhysicsScene")
    define_prim("/World/PhysicsScene", "PhysicsScene")

# --- chon prim Go2 ---
go2_prim = GO2_PRIM or (roots[0] if roots else "/World/go2")
print(f">>> Se attach policy vao prim: {go2_prim}")
print(">>> Physics engine active:", SimulationManager.get_active_physics_engine())


# ============================================================================
# 2. CAU HINH SIM + ATTACH POLICY (giong thu tu trong go2_teleop.py)
# ============================================================================
RenderingManager.set_dt(8.0 / 200.0)
SimulationManager.set_physics_sim_device(args.device)
SimulationManager.set_physics_dt(1.0 / 200.0)

# prim /World/go2 DA TON TAI -> PolicyController se attach, KHONG spawn con thu 2
go2 = Go2Debug(prim_path=go2_prim)
print(">>> POLICY NVIDIA (mac dinh) attach vao Go2 co san trong scene")
print(">>> Spawn -> settle ~0.5s -> W/S tien-lui, A/D ngang, Q/E xoay")

base_command = torch.zeros(3, device=args.device)
first_step, reset_needed, warmup = True, False, 0


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
        go2.robot.set_dof_position_targets(positions=wp.from_torch(go2.default_pos))
        warmup += 1
        if warmup == SETTLE_STEPS:
            print(">>> Het settle — gio dieu khien bang W/S/A/D/Q/E")
    else:
        go2.forward(step_size, base_command)


SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
omni.timeline.get_timeline_interface().play()
simulation_app.update()

while simulation_app.is_running():
    simulation_app.update()
    if SimulationManager.is_simulating():
        vx = (SPEED if KB.W in keys else 0) - (SPEED if KB.S in keys else 0)
        vy = (SPEED if KB.A in keys else 0) - (SPEED if KB.D in keys else 0)
        wz = (TURN if KB.Q in keys else 0) - (TURN if KB.E in keys else 0)
        base_command = torch.tensor([float(vx), float(vy), float(wz)], device=args.device)
    else:
        reset_needed = True

simulation_app.close()
