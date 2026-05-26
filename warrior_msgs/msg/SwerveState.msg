# State of a swerve module, as reported by the topic-bridge
string  swerve_id         # "02_swerve" | "03_swerve" | "04_swerve"
float64 steer_angle       # rad
float64 drive_speed       # m/s

# Constants for target IDs
string FRONT = "02_swerve"
string LEFT  = "03_swerve"
string RIGHT = "04_swerve"