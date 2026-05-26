# Per-module command from the controller to the hardware manager.
string  swerve_id              # "front" | "left" | "right"
float64 steer_position_rad     # absolute steering angle, radians
float64 drive_velocity_rad_s   # wheel angular velocity, rad/s
builtin_interfaces/Time stamp
