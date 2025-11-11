# generate_barrels_course_scaled.py
import math

# Number of barrels
num_barrels = 50
z_height = 0.15

# Original course rectangle (meters)
original_course_points = [
    (-22, -14),  # bottom-left
    (22, -14),   # bottom-right
    (22, 14),    # top-right
    (-22, 14),   # top-left
    (-22, -14)   # back to start
]

# Scale factor to make course twice as big
scale = 2.0

# Scale the course points
course_points = [(x*scale, y*scale) for x, y in original_course_points]

# Compute segment lengths
segments = []
total_length = 0.0
for i in range(len(course_points)-1):
    x1, y1 = course_points[i]
    x2, y2 = course_points[i+1]
    length = math.hypot(x2-x1, y2-y1)
    segments.append((x1, y1, x2, y2, length))
    total_length += length

# Distance between barrels along the path
spacing = total_length / num_barrels

# Generate barrel positions along the path
barrel_positions = []
dist_along = 0.0
seg_index = 0

for i in range(num_barrels):
    # Find which segment this barrel is on
    while seg_index < len(segments) and dist_along > segments[seg_index][4]:
        dist_along -= segments[seg_index][4]
        seg_index += 1
    if seg_index >= len(segments):
        seg_index = len(segments) - 1
    x1, y1, x2, y2, seg_len = segments[seg_index]
    t = dist_along / seg_len  # interpolation factor
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)
    barrel_positions.append((x, y, z_height))
    dist_along += spacing

# Function to generate XML snippet for a barrel
def add_barrel(x, y, z, name):
    return f'''
    <include>
      <name>{name}</name>
      <uri>file://./barrel/barrel.sdf</uri>
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
    </include>
    '''

# Generate the world file
with open("generated_world.world", "w") as f:
    f.write('<?xml version="1.0"?>\n<sdf version="1.8">\n  <world name="CourseMap">\n')

    # Base map
    f.write('    <include>\n')
    f.write('      <uri>file://./model.sdf</uri>\n')
    f.write('      <pose>0 0 0 0 0 0</pose>\n')
    f.write('    </include>\n')

    # Add barrels
    for i, (x, y, z) in enumerate(barrel_positions, start=1):
        f.write(add_barrel(x, y, z, f"barrel_{i}"))

    f.write('  </world>\n</sdf>')

print(f"Generated world with {num_barrels} barrels along the scaled course in 'generated_world.world'")
