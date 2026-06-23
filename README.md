# piper_gripper_hardware

ROS 2 `ros2_control` hardware plugin for Piper gripper over raw SocketCAN, without `piper_sdk`.

## Protocol basis

- **Command CAN ID**: `0x159` (`ARM_GRIPPER_CTRL`)
- **Feedback CAN ID**: `0x2A8` (`ARM_GRIPPER_FEEDBACK`)
- Frame layout follows AgileX Piper V2 protocol (big-endian):
  - Command: `int32 angle(0.001mm)`, `uint16 effort(0.001N*m)`, `uint8 status`, `uint8 set_zero`
  - Feedback: `int32 angle(0.001mm)`, `int16 effort(0.001N*m)`, `uint8 status bits`

## Build (Jazzy)

```bash
colcon build --packages-select piper_gripper_hardware
```

## ros2_control URDF snippet

```xml
<ros2_control name="piper_gripper" type="system">
  <hardware>
    <plugin>piper_gripper_hardware/PiperGripperHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="command_can_id">345</param> <!-- 0x159 -->
    <param name="feedback_can_id">680</param> <!-- 0x2A8 -->
    <param name="min_position_m">0.0</param>
    <param name="max_position_m">0.08</param>
    <param name="initial_position_m">0.0</param>
    <param name="default_effort_mn">1000</param>
    <param name="feedback_timeout_ms">1000</param>
    <param name="command_refresh_interval_ms">100</param>
  </hardware>

  <joint name="gripper_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="effort"/>
    <state_interface name="status_code"/>
    <state_interface name="enabled"/>
    <state_interface name="homed"/>
    <state_interface name="fault"/>
  </joint>
</ros2_control>
```

## Runtime behavior

- Activate sends `status=0x03` (enable + clear error), then periodic `status=0x01` commands.
- Deactivate sends `status=0x00` (disable).
- Command input is `position` in meters, converted to protocol unit (`1e-6 m` per count).
- Exposes feedback: position, effort, status_code, enabled, homed, fault.
