"""Select GPS fusion inputs without leaving a wheel source in encoder-free mode."""

from copy import deepcopy
import re


def fusion_parameters(configuration, use_wheel_odometry):
    """
    Return separate ROS parameter dictionaries for the GPS localization nodes.

    The mode owns wheel/twist input selection even when a custom parameters file
    is supplied. Other tuning, GPS inputs, and frame names remain configurable.
    Numbered inputs must stay contiguous: robot_localization stops reading them
    at the first missing index.
    """
    result = deepcopy(configuration)
    for name in ('ekf_node_odom', 'ekf_node_map'):
        params = result[name]['ros__parameters']
        sources = []
        for key in sorted(params, key=lambda key: (
                int(key[4:]) if re.fullmatch(r'odom\d+', key) else -1)):
            if not re.fullmatch(r'odom\d+', key):
                continue
            source = {item[len(key):]: value for item, value in params.items()
                      if item == key or item.startswith(key + '_')}
            # odom0 is the documented wheel slot in the source configuration;
            # also remove any additional /odom source supplied by the caller.
            if key != 'odom0' and source[''].lstrip('/') != 'odom':
                sources.append(source)
        for key in list(params):
            if re.fullmatch(r'odom\d+(?:_.*)?', key):
                del params[key]
        if use_wheel_odometry:
            wheel = configuration[name]['ros__parameters']
            sources.insert(0, {
                key[len('odom0'):]: value for key, value in wheel.items()
                if key == 'odom0' or key.startswith('odom0_')
            })
        for index, source in enumerate(sources):
            params.update({f'odom{index}' + suffix: value
                           for suffix, value in source.items()})
        if not use_wheel_odometry:
            params.update({
                'twist0': '/gnss/velocity_body',
                'twist0_config': [False] * 6 + [True, True] + [False] * 7,
                'twist0_queue_size': 10,
            })
    return result
