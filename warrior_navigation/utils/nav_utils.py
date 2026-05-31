

import yaml

def extract_first_coords_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    
    if 'waypoints' in data and isinstance(data['waypoints'], list) and len(data['waypoints']) > 0:
        first_waypoint = data['waypoints'][0]
        latitude = first_waypoint.get('latitude')
        longitude = first_waypoint.get('longitude')
        return latitude, longitude
    
    raise ValueError("No waypoints found in the YAML file.")