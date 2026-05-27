# Per-module feedback from the hardware manager to the controller.
string  swerve_id              # "front" | "left" | "right"

float64 steer_position_rad     # current steering angle, radians (from SPARK MAX)
float64 steer_velocity_rad_s   # current steering angular velocity, rad/s
float64 drive_velocity_rad_s   # current drive wheel angular velocity, rad/s (open-loop echo if no encoder)

bool    steer_connected
bool    drive_connected
string  steer_status           # e.g. "active", "scanning", "no_feedback", "timeout"
string  drive_status

builtin_interfaces/Time stamp
