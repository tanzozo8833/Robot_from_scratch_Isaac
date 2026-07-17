# 1. cai isaac-lab, train:
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# xem các tag/bản 3.0 có sẵn
git tag | grep -i "3.0"
git branch -a

# lấy dòng 3.0 cho Isaac Sim 6.0 — chọn 1 trong 2:
git checkout develop 



        1. To activate the environment, run:                conda activate env_isaaclab
		2. To install Isaac Lab extensions, run:            isaaclab.sh -i
		3. To perform formatting, run:                      isaaclab.sh -f
		4. To deactivate the environment, run:              conda deactivate


# 1.1 setup conda env
--copy paste vao terminal

cat > ~/isaacsim/setup_conda_env.sh << 'EOF'
#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export CARB_APP_PATH=$SCRIPT_DIR/kit
export ISAAC_PATH=$SCRIPT_DIR
export EXP_PATH=$SCRIPT_DIR/apps
source ${SCRIPT_DIR}/setup_python_env.sh
EOF

tiep theo: - tìm kiếm xem có những bản nào của go2 
cd ~/IsaacLab
./isaaclab.sh -p scripts/environments/list_envs.py | grep -i -E "go2|velocity"

output: 
|  159   | Isaac-Velocity-Flat-UnitreeGo2                                                 | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.flat_env_cfg:UnitreeGo2FlatEnvCfg                                                     |
|  160   | Isaac-Velocity-Flat-UnitreeGo2-Play                                            | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.flat_env_cfg:UnitreeGo2FlatEnvCfg_PLAY                                                |
|  161   | Isaac-Velocity-Rough-UnitreeGo2                                                | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.rough_env_cfg:UnitreeGo2RoughEnvCfg                                                   |
|  162   | Isaac-Velocity-Rough-UnitreeGo2-Play                                           | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.rough_env_cfg:UnitreeGo2RoughEnvCfg_PLAY|  159   | Isaac-Velocity-Flat-UnitreeGo2                                                 | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.flat_env_cfg:UnitreeGo2FlatEnvCfg                                                     |
|  160   | Isaac-Velocity-Flat-UnitreeGo2-Play                                            | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.flat_env_cfg:UnitreeGo2FlatEnvCfg_PLAY                                                |
|  161   | Isaac-Velocity-Rough-UnitreeGo2                                                | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.rough_env_cfg:UnitreeGo2RoughEnvCfg                                                   |
|  162   | Isaac-Velocity-Rough-UnitreeGo2-Play                                           | isaaclab.envs:ManagerBasedRLEnv                                                                               | isaaclab_tasks.core.velocity.configgo2.rough_env_cfg:UnitreeGo2RoughEnvCfg_PLAY


trong đó:
Isaac-Velocity-Flat-UnitreeGo2 (#159) — đi mặt phẳng
Isaac-Velocity-Rough-UnitreeGo2 (#161) — đi địa hình gồ ghề

# 1.2 để train chạy command sau: 
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-UnitreeGo2 --headless --num_envs 4096


có thể có bug tương thích giữa Python bản conda-forge khi train
fix: khởi tạo file sitecustomize.py

cat > /home/azureuser/anaconda3/envs/env_isaaclab/lib/python3.12/site-packages/sitecustomize.py << 'EOF'
import re
import platform

_orig = platform._sys_version

def _patched(sys_version=None):
    import sys as _sys
    v = sys_version if sys_version is not None else _sys.version
    # bỏ đoạn "| packaged by conda-forge |" mà conda chèn vào
    v = re.sub(r"\s*\|\s*packaged by[^|]*\|\s*", " ", v)
    return _orig(v)

platform._sys_version = _patched
EOF

kiểm tra: copy paste to terminal

/home/azureuser/anaconda3/envs/env_isaaclab/bin/python -c "import platform; print(platform.python_implementation(), platform.python_version())"

sau đó chạy lại train, tại sao cần train:

Vấn đề: con Go2 không có bánh xe. Nó có 12 khớp động cơ ở 4 chân. Không có công thức đơn giản nào biến "đi 0.5 m/s" thành 12 góc khớp phối hợp nhịp nhàng để robot vừa bước tới vừa không ngã.


Robot bánh xe (TurtleBot...): /cmd_vel map thẳng ra tốc độ 2 bánh → cắm Nav2 vào là chạy, không cần train gì. Đó là lý do đa số tutorial Nav2 dùng robot bánh xe.
Robot 4 chân (Go2): giữa /cmd_vel và cái chân bắt buộc phải có một lớp policy đứng giữa để biến lệnh thành bước chân. Đây là mảnh mà bạn không thể lấy free.

khi train:
4096 con Go2 ảo chạy song song trên GPU, mỗi con thử bước đi theo policy hiện tại (ban đầu là ngẫu nhiên → ngã liên tục).
Mỗi bước, môi trường chấm điểm (reward): đi đúng tốc độ lệnh /cmd_vel → +điểm; ngã, lắc lư, tốn năng lượng → −điểm.
Thuật toán PPO điều chỉnh trọng số mạng neural để lần sau được điểm cao hơn.
Lặp lại. Nhờ 4096 con chạy song song, robot tích lũy kinh nghiệm cực nhanh → 6 phút là đủ đi vững trên mặt phẳng.

check policy đã có chưa:

LOGDIR=$(ls -dt ~/IsaacLab/logs/rsl_rl/*/*/ | head -1)
echo $LOGDIR
ls -la $LOGDIR
ls -la $LOGDIR/exported/

output cần có: file quan trọng: policy.pt 
(env_isaaclab) azureuser@issas-sim-a10-u24-vm:~/IsaacLab ls -la $LOGDIR/exported/
drwxrwxr-x 2 azureuser azureuser   4096 Jul  2 17:16 .
drwxrwxr-x 5 azureuser azureuser   4096 Jul  2 17:16 ..
-rw-rw-r-- 1 azureuser azureuser  11345 Jul  2 17:19 policy.onnx
-rw-rw-r-- 1 azureuser azureuser 163328 Jul  2 17:19 policy.onnx.data
-rw-rw-r-- 1 azureuser azureuser 175532 Jul  2 17:19 policy.pt

lưu policy ra folder riêng:
mkdir -p ~/go2_deploy
cp $LOGDIR/exported/policy.pt ~/go2_deploy/go2_flat_policy.pt
ls -la ~/go2_deploy/

folder structure:
azureuser@issas-sim-a10-u24-vm:~$ ls
go2_deploy  IsaacLab  isaacsim    tan: trong này có .usd của môi trường có robot go2 -> có thể sim được 

flow:

(./isaac-sim/image1.png)

## tổng kết 
Về môi trường, bạn đã xác minh: Ubuntu 24.04 + ROS2 Jazzy + Isaac Sim 6.0.1, và điều may mắn là cả Jazzy lẫn Isaac Sim đều chạy Python 3.12.13 → không dính lỗi rclpy mismatch vốn hành hạ người dùng bản cũ. GPU A10-24Q (24GB) đủ dùng.

Về cài đặt, bạn đã: cài Isaac Lab 3.0 beta (branch develop, bản khớp Isaac Sim 6.0) vào conda env env_isaaclab; vá 2 lỗi phát sinh (thiếu setup_conda_env.sh và bug parse version của Python conda-forge qua sitecustomize.py).
## train (./isaac-sim/image.png)
Về train, bạn đã train xong policy locomotion cho Go2 trên mặt phẳng (300 vòng, ~6 phút) → ra model_299.pt, rồi export thành policy.pt + policy.onnx. File đã copy về ~/go2_deploy/go2_flat_policy.pt. Đây là thành quả cốt lõi — "bộ não biết đi" của Go2.

Đầu vào là gì (observation = 48 số)
Đây chính là con số in_features=48 bạn thấy. Mỗi bước, môi trường đưa cho policy một vector 48 số mô tả "robot đang thế nào", gồm:

Vận tốc thân robot — tịnh tiến (3) + xoay (3)
Vector trọng lực chiếu theo thân (3) — cho robot biết nó đang nghiêng hay thẳng
Lệnh vận tốc mong muốn (3) — chính là cmd_vel: đi tới bao nhiêu, sang ngang bao nhiêu, xoay bao nhiêu
Góc 12 khớp (12) + tốc độ 12 khớp (12)
Hành động ở bước trước (12)

Cộng lại: 3+3+3+3+12+12+12 = 48. Điểm cực quan trọng: cmd_vel là một phần của đầu vào. Robot được "nhìn thấy" lệnh vận tốc ngay trong observation — đây là lý do sau này Nav2 gửi /cmd_vel thì policy phản ứng được, vì nó đã được huấn luyện để đọc và bám theo con số đó.
Đầu ra (action) là 12 số — mục tiêu góc cho 12 khớp chân, đẩy vào bộ điều khiển PD của từng khớp.
Mục tiêu train là gì (reward)
"Mục tiêu" trong RL = hàm reward. Với task Flat-Velocity của Go2, mỗi bước môi trường chấm điểm bằng tổng có trọng số của nhiều thành phần. Thưởng và phạt:
Thưởng (+): đi đúng vận tốc được lệnh — sai số giữa vận tốc thực và cmd_vel càng nhỏ, điểm càng cao. Đây là thành phần chính, định nghĩa "thế nào là tốt". Cộng thêm thưởng nhỏ cho nhịp chân hợp lý (feet air time).
Phạt (−): ngã/thân chạm đất, lắc lư theo phương thẳng đứng, nghiêng thân, tốn nhiều mô-men, giật cục (action đổi quá gấp giữa 2 bước). Những cái phạt này chính là thứ khiến policy học được dáng đi mượt và không ngã, chứ không phải giật đùng đùng.
Kèm theo là điều kiện kết thúc episode: nếu robot ngã (thân chạm đất) → cắt, reset về đứng thẳng, học lại. Nhờ vậy 4096 con thử–ngã–reset liên tục, tích lũy kinh nghiệm cực nhanh.

thuc te:
Thu tu 12 khop: ['FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint', 'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint', 'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint']
>>> default_pos: ['+0.10', '-0.10', '+0.10', '-0.10', '+0.80', '+0.80', '+1.00', '+1.00', '-1.50', '-1.50', '-1.50', '-1.50']
Module isaacsim.core.experimental.utils.impl.ops 762c1cf load on device 'cuda:0' took 6.97 ms  (cached)
--- obs #50 ---
 base_lin_vel   [+0.02 -0.01 +0.02]
 base_ang_vel   [+0.44 +0.00 -0.15]
 proj_gravity   [-0.00 -0.02 +1.00]   nen ~ [+0.00 +0.00 -1.00]
 vel_command    [+0.00 +0.00 +0.00]   = lenh WASD cua ban
 jointpos-def   [+0.05 +0.07 -0.09 +0.01 -0.37 -0.03 +0.17 +0.15 -0.18 -0.24 -0.15 -0.21]   nen ~ 0 luc vua spawn
 joint_vel      [-0.84 -1.71 -0.75 -0.45 -1.48 -1.21 -4.26 +2.22 +3.24 -2.08 +2.25 -1.23]
 prev_action    [+0.74 -0.03 -0.50 -0.12 -0.89 -0.43 +0.24 +1.03 -0.34 -1.13 -0.51 -0.70]

./python.sh ~/go2_deploy/go2_teleop.py --device cuda
chạy robot với policy 



# cần tạo ra 1 policy cho con unitree go2 

about RL and PPO:

Reinforce: Simple and easy to understand but often unstable due to high variance in updates. PPO improves stability by limiting how much the policy can change at each step.

about RL:
	- learning through experience to make good decision  