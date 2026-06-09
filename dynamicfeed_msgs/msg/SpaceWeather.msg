# Space weather — relevant to GNSS accuracy, HF comms, and aerial/orbital robots.
std_msgs/Header header
float64 kp_index           # geomagnetic Kp (0..9)
uint8 storm_g              # G-scale geomagnetic storm (0..5)
uint8 storm_s              # S-scale radiation storm (0..5)
Provenance provenance
