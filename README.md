# Go2 RL Agents
## topic /sportmodestate
Type: unitree_go/msg/SportModeState
---
TimeSpec stamp
	int32 sec
	uint32 nanosec
uint32 error_code
IMUState imu_state
	float32[4] quaternion
	float32[3] gyroscope
	float32[3] accelerometer
	float32[3] rpy
	int8 temperature
uint8 mode
float32 progress
uint8 gait_type
float32 foot_raise_height
float32[3] position
float32 body_height
float32[3] velocity
float32 yaw_speed
float32[4] range_obstacle
int16[4] foot_force
float32[12] foot_position_body
float32[12] foot_speed_body
---
foot_force:
    - FR
    - FL
    - RR
    - RL
velocity
imu_state
    - gyroscope
    - accelerometer



## topic /lowstate
Type: unitree_go/msg/LowState

uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
IMUState imu_state
	float32[4] quaternion
	float32[3] gyroscope
	float32[3] accelerometer
	float32[3] rpy
	int8 temperature
MotorState[20] motor_state
	uint8 mode
	float32 q
	float32 dq
	float32 ddq
	float32 tau_est
	float32 q_raw
	float32 dq_raw
	float32 ddq_raw
	int8 temperature
	uint32 lost
	uint32[2] reserve
BmsState bms_state
	uint8 version_high
	uint8 version_low
	uint8 status
	uint8 soc
	int32 current
	uint16 cycle
	int8[2] bq_ntc
	int8[2] mcu_ntc
	uint16[15] cell_vol
int16[4] foot_force
int16[4] foot_force_est
uint32 tick
uint8[40] wireless_remote
uint8 bit_flag
float32 adc_reel
int8 temperature_ntc1
int8 temperature_ntc2
float32 power_v
float32 power_a
uint16[4] fan_frequency
uint32 reserve
uint32 crc

motor_state:
    - FR hip_x, hip_y, calf
    - FL hip_x, hip_y, calf
    - RR hip_x, hip_y, calf
    - RL hip_x, hip_y, calf


## topic /lowcmd
Type: unitree_go/msg/LowCmd

uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
MotorCmd[20] motor_cmd
	uint8 mode
	float32 q
	float32 dq
	float32 tau
	float32 kp
	float32 kd
	uint32[3] reserve
BmsCmd bms_cmd
	uint8 off
	uint8[3] reserve
uint8[40] wireless_remote
uint8[12] led
uint8[2] fan
uint8 gpio
uint32 reserve
uint32 crc
