# Bus Measurement
float32 bus_current         # A Current drawn from the DC bus by the controller
float32 bus_voltage         # V DC bus voltage at the controller

# Velocity Measurement
float32 motor_velocity      # rpm Motor angular frequency in revolutions per minute

# Steering Feedback
uint16 actual_steering     # Range: 0-3600 Actual steering value received as feedback

# Heat-sink & Motor Temperature Measurement
float32 heat_sink_temp      # °C Internal temperature of heat-sink (case)
float32 motor_temp          # °C Internal temperature of motor

# Flags for drive mode and enums 
bool manual_mode            # true when controlled by steering wheel

bool PC_MODE=false
bool STEERING_MODE=true