
**Right Arm**
pixi run robot

cd ~/projects/pixi_franka_crisp_ws
pixi run -e humble ros2 service call /franka_right/gripper/dynamixel_hardware_interface/set_dxl_torque std_srvs/srv/SetBool "{data: false}"

pixi run -e humble python src/crisp_py/scripts/gripper_manual_calibration.py --namespace franka_right --joint-state-topic franka_gripper/joint_states --config-file gripper_right.yaml

pixi run -e humble ros2 service call /franka_right/gripper/dynamixel_hardware_interface/set_dxl_torque std_srvs/srv/SetBool "{data: true}"


**Left Arm**

cd ~/projects/pixi_franka_crisp_ws
pixi run -e humble ros2 service call /franka_left/gripper/dynamixel_hardware_interface/set_dxl_torque std_srvs/srv/SetBool "{data: false}"

pixi run -e humble python src/crisp_py/scripts/gripper_manual_calibration.py --namespace franka_left --joint-state-topic franka_gripper/joint_states --config-file gripper_left.yaml


pixi run -e humble ros2 service call /franka_left/gripper/dynamixel_hardware_interface/set_dxl_torque std_srvs/srv/SetBool "{data: true}"
