# BunkerStatus
Header header

# base state
uint8 base_state
uint8 control_mode
uint8 fault_code
float64 battery_voltage

float32 vx
float32 vth

# motor state
BunkerMotorState[2] motor_states

# driver state
BunkerDriverState[2] driver_states
