# Desired throttle and steering commands for the vehicle
std_msgs/Header header   # stamp = command compute time; frame_id="base_link"
float32 throttle         # normalized 0.0–1.0, consistent with VCU pedal map
float32 steering         # normalized 0.0–1.0, consistent with low-level steering controller
