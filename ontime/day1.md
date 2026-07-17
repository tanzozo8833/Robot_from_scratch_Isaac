# Kiến trúc hệ thống điều khiển Robot (3 lớp + Bridge)

## 0. Nguyên tắc thiết kế

Hệ thống tách thành 3 lớp theo mức độ "gần phần cứng" và **tần số điều khiển (control rate)**:

- **Layer 1 – UI/Simulation:** con người tương tác, cấu hình, giám sát (chậm, event-driven).
- **Layer 2 – High-level Planning (MATLAB):** tính toán *offline / design-time*, sinh dữ liệu tĩnh và mã robot.
- **Bridge – ROS 2:** perception + motion *real-time mềm* (soft real-time, ~10–100 Hz), keo dán giữa Layer 2 và Layer 3.
- **Layer 3 – Robot SDK / Firmware:** điều khiển motor *real-time cứng* (hard real-time, 1 kHz–20 kHz).

> **Ranh giới quan trọng (điểm dễ sai nhất):** MATLAB **không** nằm trong vòng lặp real-time. MATLAB lo phần *tĩnh/offline* (quỹ đạo tham chiếu, tối ưu, sinh mã). ROS lo phần *động/online* (kiểm tra va chạm với sensor thật, local re-planning). Xem mục 8.

---

## 1. Layer 1 — UI & Simulation

Ba công cụ hiển thị phục vụ **mục đích khác nhau**, không đồng nhất:

| Công cụ | Vai trò thực tế | Dành cho |
|---|---|---|
| **React / Node.js** | Dashboard: monitoring, configuration, user control | End-user / operator |
| **Unreal Engine** | Real-time 3D rendering, **Digital Twin** (HMI đẹp, đồng bộ trạng thái robot thật) | Operator / demo |
| **Gazebo** | **Physics simulation** (mô phỏng vật lý, test thuật toán trước khi chạy thật) | Kỹ sư / CI test |
| **RViz** | Debug visualization của ROS (TF, pointcloud, planning scene) | **Developer** (không phải UI cho user cuối) |

- **CAD Integration:** cho phép user tùy biến robot (limbs, sensors). CAD → URDF/SDF để cả Gazebo, RViz và MoveIt cùng dùng chung mô hình.
- **Digital Twin loop:** trạng thái robot thật (joint states, pose) phải chảy ngược từ ROS → Unreal để twin phản ánh đúng thực tế (đây là feedback path còn thiếu trong bản gốc).

**Giao thức Layer 1 ↔ Backend:** WebSocket (real-time telemetry) + REST API (config/command). Payload: JSON.

---

## 2. Layer 2 — High-level Planning & Code Generation (MATLAB)

Chạy **offline / design-time** trên IPC. Nhiệm vụ:

- Motion planning cấp cao: **path planning, trajectory generation, smoothing, time scheduling**.
- **Kinematic solvers** (IK/FK) cho cấu hình robot cụ thể.
- Collision check **với world tĩnh đã biết** (a-priori), sinh ra quỹ đạo tham chiếu collision-free.
- Output = **dữ liệu tĩnh (static data):** quỹ đạo tham chiếu, waypoint, time profile.

**Code Generation Engine:** sinh mã cho phần cứng đích:
- Robot công nghiệp: **TP / LS / Karel** (Fanuc).
- Máy CNC: **G-code**.

> **Khuyến nghị (fix quan trọng):** không gọi trực tiếp *MATLAB Engine* trong runtime (nặng, latency cao, phụ thuộc license khi deploy). Dùng **MATLAB Coder** để biên dịch thuật toán planning thành **node C/C++** nhúng thẳng vào ROS. Khi đó BT gọi một node C++ thuần, không cần MATLAB runtime chạy song song.

---

## 3. Bridge — ROS 2 (Perception + Real-time Motion)

Cầu nối Layer 2 ↔ Layer 3, trao đổi dữ liệu qua **topics / services / actions**.

- **Motion planning & execution real-time:**
  - **MoveIt** (+ **OMPL**): cho **robot tay máy (arm)** — planning trong không gian cấu hình, kiểm tra va chạm online.
  - **Nav2:** chỉ dùng khi có **base di động (AMR/mobile robot)**. Nếu hệ thống chỉ có cánh tay cố định + CNC thì **không cần Nav2**.
- **Perception:** **CV, SLAM** — xử lý dữ liệu lidar/camera để mapping môi trường (octomap / costmap).
- **Kết hợp dữ liệu:** static data từ Layer 2 (**quỹ đạo tham chiếu**) **+** dữ liệu real-time (lidar/camera) → **local re-planning** và **exception handling** khi có vật cản động.

**Giao tiếp UI ↔ ROS:** `rosbridge_suite` chuyển JSON/WebSocket ↔ ROS message. (Đây là "ROS Bridge" trong sơ đồ; **khác** với shared memory — shared memory chỉ dùng cho trao đổi tốc độ cao trong cùng máy, không phải cho UI.)

---

## 4. Orchestration — Behavior Tree trên IPC

Thay **FSM** (và logic if-else kiểu PLC truyền thống) bằng **Behavior Tree** để logic có cấu trúc, tái sử dụng, dễ mở rộng:

- Mỗi task = một **node** độc lập, có thể tái sử dụng giữa các cây.
- Thư viện node viết bằng **C++/Python**; runtime dùng **BehaviorTree.CPP** đọc file XML.
- Visualize / debug cây bằng **Groot2**.

### Behavior Tree XML (đã sửa)

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Fallback name="RootFallback">
      <Sequence name="NormalOperation">
        <Condition ID="IsPartAvailable"/>       <!-- Có phôi/part để xử lý không? -->
        <Action    ID="ComputeTrajectory"/>     <!-- Layer 2: lấy quỹ đạo (node C++ do MATLAB Coder sinh) -->
        <Condition ID="CheckCollisionROS"/>     <!-- ROS/MoveIt: kiểm tra va chạm REAL-TIME với data lidar/camera -->
        <Action    ID="ExecuteMoveROS"/>        <!-- Bridge: gửi setpoint xuống Layer 3 -->
      </Sequence>
      <Action ID="SafeRecovery"/>               <!-- Fail ở bất kỳ bước nào -> về vị trí an toàn (home) -->
    </Fallback>
  </BehaviorTree>
</root>
```

> **Sửa so với bản gốc:**
> 1. Bản gốc để `-- call layer 2 --` là **text thô trong cây → XML không hợp lệ**. Đã chuyển thành comment `<!-- -->`.
> 2. Tách **`CheckCollisionROS`** riêng: va chạm real-time query từ **ROS/MoveIt (planning scene + sensor)**, **không** query MATLAB. MATLAB chỉ cho quỹ đạo tĩnh a-priori; ROS mới kiểm tra với thực tế động.

**Vai trò 2 node chính:**
- `CheckCollisionROS`: kiểm tra tọa độ/quỹ đạo có an toàn không, dựa trên **costmap/octomap real-time** (dữ liệu sensor), không phải chỉ dữ liệu tĩnh.
- `ExecuteMoveROS`: nếu an toàn → gửi lệnh điều khiển qua **Bridge** xuống STM32/Fanuc.

---

## 5. Layer 3 — Robot SDK / Firmware

### Nhánh 1 — Custom robot (tự thiết kế)
- **C/C++ + FreeRTOS trên STM32**: real-time motor control, sensor integration.
- STM32 nhận **setpoint** (position/velocity) từ IPC → chạy vòng **PID** điều khiển motor (current/velocity/position loop ở tần số cao, ví dụ current loop 10–20 kHz).
- IPC là "bộ não" ra quyết định; STM32 trực tiếp điều khiển "cơ bắp" (motor) và thu thập dữ liệu cảm biến.

### Nhánh 2 — Industrial robot
- **Fanuc Controller** chạy chương trình **TP / Karel** (do Layer 2 sinh ra).
- IPC/ROS **không** điều khiển motor cấp thấp — bộ điều khiển Fanuc lo hết nội bộ; IPC chỉ gửi lệnh cấp cao.
- Tích hợp qua **ROS-Industrial** (`fanuc` driver: socket TCP + Karel server chạy trên controller) hoặc Ethernet/IP, PROFINET.

Cả hai nhánh giao tiếp với robot vật lý qua **Robot Controller** tương ứng.

---

## 6. Hardware & Communication

### IPC / PLC
- **IPC (Industrial PC):** chạy ROS 2, Backend Node.js, (MATLAB Coder-generated nodes), Behavior Tree. Là master điều phối.
- **PLC (tùy chọn):** giữ lại cho **safety I/O phần cứng** (E-stop, light curtain, interlock) — mạch an toàn phải độc lập với phần mềm. Logic *nghiệp vụ* thì chuyển lên Behavior Tree; PLC chỉ còn lo an toàn.

### Giao tiếp IPC ↔ STM32 (điểm cần thống nhất)
Bản gốc vừa nói *CAN Bus/Ethernet/Serial* vừa nói *EtherCAT* — cần chọn một. Khuyến nghị theo yêu cầu real-time:

| Lựa chọn | Ưu | Lưu ý |
|---|---|---|
| **EtherCAT** (khuyến nghị nếu nhiều trục, đồng bộ cao) | Cyclic 1 kHz+, jitter thấp, đồng bộ đa trục | **STM32 không có EtherCAT native** → cần chip **ESC** (LAN9252 / ET1100) giao tiếp qua SPI |
| **CAN-FD** | STM32 hỗ trợ native, đơn giản, chi phí thấp | Băng thông thấp hơn EtherCAT, hợp cho ít trục |
| **micro-ROS** (UART/UDP) | STM32 trở thành **ROS 2 node** trực tiếp, đúng ý "STM32 nhận setpoint từ ROS node" | Không hard real-time bằng fieldbus, hợp prototype |

### Bảng giao thức tổng hợp

| Kết nối | Giao thức | Ghi chú |
|---|---|---|
| UI ↔ Backend | WebSocket / REST | JSON |
| Backend ↔ ROS 2 | rosbridge (WS→ROS) | |
| BT ↔ Layer 2 | Node C++ (MATLAB Coder) / MATLAB Engine | ưu tiên compiled |
| BT ↔ ROS nodes | ROS 2 topic/service/**action** | in-process |
| IPC ↔ STM32 | **EtherCAT** / CAN-FD / micro-ROS | real-time cyclic |
| IPC ↔ Fanuc | ROS-Industrial (TCP+Karel) / Ethernet-IP | |

---

## 7. Sơ đồ luồng runtime (đã sửa)

```
[ LAYER 1: UI ]  React · Unreal(Digital Twin) · Gazebo(sim) · RViz(debug)
      |  ^
      |  |  (WebSocket / REST — JSON)
      v  |  (telemetry: joint states, pose  ↑ feedback về Digital Twin)
======|==|===============================================================
| IPC (INDUSTRIAL PC)                                                    |
|                                                                        |
|  [ Backend Node.js ]  <--- tiếp nhận & phản hồi UI                     |
|         |                                                              |
|         v  (rosbridge)                                                 |
|  [ Behavior Tree ]  <--- quản lý control flow (thay FSM)               |
|      /            \                                                    |
|     v              v                                                   |
| [Layer 2: Planner] [ROS 2 Bridge]                                      |
|  MATLAB/Coder       - MoveIt/OMPL (arm) | Nav2 (nếu có base)           |
|  - static traj      - CV / SLAM (costmap real-time)                    |
|  - TP/Karel/G-code  - CheckCollision (sensor) → ExecuteMove            |
|         \______________/                                               |
|         static data + real-time sensor → local re-planning             |
======================================|=================================
                                      |  (EtherCAT / CAN-FD / micro-ROS)
                                      v
[ LAYER 3: EXECUTION ]
   Nhánh 1: STM32 + FreeRTOS  → PID → Motor (custom robot)
   Nhánh 2: Fanuc Controller (TP/Karel)  → Motor (industrial)
                                      |
                                      v
                          [ Robot vật lý + Sensors ]
                                      |
                     (dữ liệu sensor ↑ quay lại ROS: SLAM/collision)
```

---

## 8. Luồng dữ liệu: Design-time vs Runtime (tóm tắt ranh giới)

**Design-time (offline, MATLAB — chậm, một lần):**
CAD/URDF → path planning → smoothing → time schedule → collision check (world tĩnh) → **quỹ đạo tham chiếu + mã TP/Karel/G-code**.

**Runtime (online, ROS + BT — real-time, vòng lặp):**
BT nhận job → nạp quỹ đạo tham chiếu → **ROS kiểm tra va chạm với sensor thật** → nếu có vật cản động thì **local re-planning** → gửi setpoint xuống STM32/Fanuc → nhận feedback → cập nhật Digital Twin → lặp lại.

---

## Phụ lục — Tóm tắt các điểm đã sửa

1. **XML BT không hợp lệ** (text `-- ... --` trong cây) → chuyển sang comment `<!-- -->`; nâng lên `BTCPP_format="4"` (Groot2).
2. **MATLAB trong vòng real-time** → tách rõ: MATLAB = offline/static; ROS = online collision check. Node `CheckCollision` query **ROS/sensor**, không query MATLAB. Gợi ý **MATLAB Coder** để deploy không cần MATLAB runtime.
3. **RViz ≠ UI người dùng** → RViz là công cụ debug của developer; tách vai trò Unreal (twin) / Gazebo (physics) / RViz (debug).
4. **Nav2 vs MoveIt** → Nav2 chỉ khi có base di động; arm dùng MoveIt/OMPL.
5. **Mâu thuẫn giao thức STM32** (CAN/Serial/Ethernet vs EtherCAT) → thống nhất, kèm lưu ý **STM32 cần chip ESC** cho EtherCAT, hoặc dùng CAN-FD / micro-ROS.
6. **"ROS Bridge / Shared Memory"** → làm rõ là hai thứ khác nhau (rosbridge cho UI; shared memory cho intra-host tốc độ cao).
7. **Digital Twin thiếu feedback** → thêm luồng trạng thái robot thật quay về Unreal.
8. **Vai trò PLC** → làm rõ chỉ dùng cho safety I/O phần cứng; logic nghiệp vụ chuyển lên BT.
9. **Fanuc integration** → bổ sung ROS-Industrial làm cầu nối chính thức.