#!/usr/bin/env python3
"""Compile one georeferenced course into shared geometry and GPS missions."""

import argparse
import json
import math
from pathlib import Path
import random
import sys

from pyproj import Transformer
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
# Also support importlib loading by the physical-contract test suite.
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from site_model import SiteModel, curb_mesh


def geographic_transform(datum):
    if not math.isclose(float(datum.get('heading_deg', 0)), 0, abs_tol=1.e-12):
        raise ValueError('datum.heading_deg must be 0 for ENU; rotate placement.heading_deg instead')
    return Transformer.from_pipeline(
        '+proj=pipeline +step +proj=cart +ellps=WGS84 '
        '+step +proj=topocentric +ellps=WGS84 '
        f'+lat_0={datum["latitude"]} +lon_0={datum["longitude"]} '
        f'+h_0={datum["elevation"]}')


def smooth_loop(control, samples=24):
    """Periodic uniform cubic B-spline; no external mesh/spline dependency."""
    output = []
    for i in range(len(control)):
        p = [control[(i + j) % len(control)] for j in range(4)]
        for step in range(samples):
            t = step / samples
            weights = [(1-t)**3, 3*t**3-6*t*t+4,
                       -3*t**3+3*t*t+3*t+1, t**3]
            output.append([sum(weights[j] * p[j][axis] for j in range(4))/6
                           for axis in range(2)])
    return output


def length(points, closed=False):
    return sum(math.dist(a, b) for a, b in zip(
        points, points[1:] + ([points[0]] if closed else [])))


def pose(x, y, z=0.0, yaw=0.0):
    return [x, y, z, 0.0, 0.0, yaw]


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        if (a[1] > y) != (b[1] > y):
            hit = (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]
            if x < hit:
                inside = not inside
    return inside


def triangulate_polygon(points):
    """Ear clipping preserves the surveyed-map outline's concave corners."""
    ids = list(range(len(points)))
    area = sum(a[0]*b[1]-b[0]*a[1]
               for a, b in zip(points, points[1:]+points[:1]))
    if area < 0:
        ids.reverse()
    triangles = []

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

    while len(ids) > 3:
        for k in range(len(ids)):
            a, b, c = ids[k-1], ids[k], ids[(k+1) % len(ids)]
            if cross(points[a], points[b], points[c]) <= 1.e-8:
                continue
            if any(all(cross(points[u], points[v], points[j]) >= -1.e-8
                       for u, v in [(a,b), (b,c), (c,a)])
                   for j in ids if j not in (a,b,c)):
                continue
            triangles.append([a, b, c])
            del ids[k]
            break
        else:
            raise ValueError('Parking outline cannot be triangulated')
    triangles.append(ids)
    return triangles


def build(config, boundary):
    datum, course = config['datum'], config['course']
    center = config['placement']['center_enu']
    heading = math.radians(config['placement']['heading_deg'])
    raw = smooth_loop(course['control_points'])
    scale = course['target_centerline_length_m'] / length(raw, True)
    cosine, sine = math.cos(heading), math.sin(heading)

    def world(x, y):
        return [center[0] + scale*(cosine*x-sine*y),
                center[1] + scale*(sine*x+cosine*y)]

    def physical_offset(origin, east, north):
        """Rotate a physical metre offset without applying schematic scale."""
        return [origin[0]+cosine*east-sine*north,
                origin[1]+sine*east+cosine*north]

    path = [world(x, y) for x, y in raw]
    materials = {
        'asphalt': {'color': [1,1,1,1], 'roughness': .92,
                    'albedo': 'assets/asphalt_albedo.png', 'tile_size_m': 4.0},
        'tape': {'color': [.94,.94,.90,1], 'roughness': .5},
        'parking_yellow': {'color': [.50,.43,.20,1], 'roughness': .92},
        'orange': {'color': [.94,.21,.025,1], 'roughness': .48},
        'red': {'color': [.68,.035,.02,1], 'roughness': .52},
        'green': {'color': [.035,.36,.15,1], 'roughness': .52},
        'blue': {'color': [.025,.20,.55,1], 'roughness': .52},
        'reflector': {'color': [.80,.82,.79,1], 'roughness': .3, 'metallic': .08},
        'rubber': {'color': [.025,.027,.028,1], 'roughness': .88},
        'metal': {'color': [.36,.38,.40,1], 'roughness': .4, 'metallic': .65},
        'concrete': {'color': [.55,.54,.50,1], 'roughness': .95},
        'grass': {'color': [.19,.28,.085,1], 'roughness': 1},
        'bark': {'color': [.21,.14,.075,1], 'roughness': 1},
        'leaf': {'color': [.09,.22,.055,1], 'roughness': .95},
        'ramp': {'color': [.25,.29,.23,1], 'roughness': .9},
        'canopy': {'color': [.83,.84,.79,1], 'roughness': .8},
        'drum_scuff': {'color': [.27,.25,.22,1], 'roughness': .96},
        'sandbag': {'color': [.075,.073,.063,1], 'roughness': 1},
    }
    # Small deterministic variations represent sun-faded plastic and foliage,
    # not recovered photographic textures or a survey of individual objects.
    for color in ('orange', 'red', 'green', 'blue'):
        for variant, fade in enumerate((.89, .95, 1.0, 1.035)):
            rgb = materials[color]['color'][:3]
            materials[f'{color}_aged_{variant}'] = {
                'color': [min(1, component*fade + .012*variant)
                          for component in rgb] + [1], 'roughness': .56+.05*variant}
    for variant, rgb in enumerate(((.070,.155,.030), (.105,.205,.041),
                                    (.155,.255,.055), (.115,.240,.050),
                                    (.190,.285,.070), (.080,.180,.040))):
        materials[f'leaf_{variant}'] = {'color': [*rgb,1], 'roughness': .98}
    objects = []

    def appearance_rng(name):
        # Do not consume the route-placement RNG when adding visual details.
        key = sum((index+1)*ord(char) for index,char in enumerate(name))
        return random.Random(config['appearance']['seed']*1000003 + key)

    def obj(name, kind, transform, material, collision=False, role='context', **geometry):
        result = dict(id=name, type=kind, pose=transform, material=material,
                      collision=collision, role=role, **geometry)
        objects.append(result)
        return result

    def ribbon(name, points, width, material, z=.012, closed=False, role='lane'):
        vertices, triangles, uvs = [], [], []
        if role == 'gate':
            refined = [points[0]]
            for a,b in zip(points,points[1:]):
                steps = max(1, math.ceil(math.dist(a,b)/.25))
                refined.extend([[a[k]+(b[k]-a[k])*i/steps for k in range(2)]
                                for i in range(1,steps+1)])
            points = refined
        for a, b in zip(points, points[1:] + ([points[0]] if closed else [])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            norm = math.hypot(dx, dy)
            if norm < 1.e-8:
                continue
            nx, ny = -dy/norm*width/2, dx/norm*width/2
            index = len(vertices)
            vertices += [[a[0]+nx,a[1]+ny,z], [a[0]-nx,a[1]-ny,z],
                         [b[0]-nx,b[1]-ny,z], [b[0]+nx,b[1]+ny,z]]
            triangles += [[index,index+1,index+2], [index,index+2,index+3]]
            uvs += [[0,0],[1,0],[1,norm],[0,norm]]
        if vertices:
            return obj(name, 'mesh', pose(0,0), material, role=role,
                       vertices=vertices, triangles=triangles, uvs=uvs)

    converter = geographic_transform(datum)
    site = SiteModel(HERE/'reference', converter)
    pavement = [list(converter.transform(lon, lat, datum['elevation'])[:2])
                for lon, lat in boundary['coordinates'][0]]
    if pavement[-1] == pavement[0]:
        pavement.pop()
    materials['site_aerial'] = dict(color=[1,1,1,1], roughness=.95,
                                    albedo='reference/p37_oakland_2025.png')
    ground = obj('P37_asphalt', 'mesh', pose(0,0), 'site_aerial', True, 'ground',
                 **site.mesh(pavement, triangulate_polygon(pavement)))
    site.set_surface(ground)
    obj('site_terrain', 'mesh', pose(0,0), 'site_aerial', True, 'terrain',
        **site.surround(pavement, triangulate_polygon(pavement)))
    for i, (a,b) in enumerate(zip(pavement,pavement[1:]+pavement[:1])):
        obj(f'curb_{i}', 'mesh', pose(0,0), 'concrete', True, 'curb',
            **curb_mesh(a,b))
    # Parking stripes and crack repairs are captured in the georeferenced aerial.

    half=course['lane_width_m']/2
    left, right = [], []
    lane_sections=[]
    current=[]
    nml_mask=[]
    for i, point in enumerate(path):
        before, after = path[i-1], path[(i+1)%len(path)]
        dx,dy=after[0]-before[0],after[1]-before[1]
        norm=math.hypot(dx,dy)
        normal=(-dy/norm,dx/norm)
        left.append([point[0]+half*normal[0],point[1]+half*normal[1]])
        right.append([point[0]-half*normal[0],point[1]-half*normal[1]])
        nml = raw[i][0] < -14.1 and abs(raw[i][1]) < 12.5
        nml_mask.append(nml)
        if nml:
            if current: lane_sections.append(current); current=[]
        else: current.append(i)
    if current: lane_sections.append(current)
    if len(lane_sections)>1 and not nml_mask[0] and not nml_mask[-1]:
        lane_sections=[lane_sections[-1]+lane_sections[0], *lane_sections[1:-1]]
    for j, section in enumerate(lane_sections):
        for side, points in [('left',left),('right',right)]:
            ribbon(f'tape_{side}_{j}', [points[i] for i in section],
                   course['tape_width_m'], 'tape')

    barrel_centers=[]
    def barrel(name, x, y, color='orange'):
        appearance = appearance_rng(name)
        body_color = f'{color}_aged_{appearance.randrange(4)}'
        barrel_centers.append([x,y])
        obj(name+'_body','cylinder',pose(x,y,.46),body_color,True,'barrel',radius=.29,length=.82)
        obj(name+'_foot','cylinder',pose(x,y,.035),'rubber',True,'barrel',radius=.36,length=.07)
        obj(name+'_lid','cylinder',pose(x,y,.88),'rubber',False,'barrel',radius=.285,length=.025)
        obj(name+'_bung','cylinder',pose(*physical_offset([x,y],.12,-.08),.903),'rubber',False,'barrel',
            radius=.027,length=.018)
        for j,z in enumerate([.30,.63]):
            obj(name+f'_reflector_{j}','cylinder',pose(x,y,z),'reflector',False,'barrel',
                radius=.294,length=.16)
        for j,z in enumerate([.12,.45,.80]):
            obj(name+f'_rib_{j}','cylinder',pose(x,y,z),body_color,False,'barrel',
                radius=.303,length=.025)
        # Dark wear marks follow the drum surface and remain inside its foot.
        vertices, triangles = [], []
        for _ in range(7):
            angle = appearance.uniform(0,math.tau)
            da = appearance.uniform(.012,.055)
            z = appearance.uniform(.16,.76)
            height = appearance.uniform(.004,.018)
            start = len(vertices)
            vertices += [[.295*math.cos(a),.295*math.sin(a),zz]
                         for a,zz in [(angle-da,z),(angle+da,z),
                                      (angle+da,z+height),(angle-da,z+height)]]
            triangles += [[start,start+1,start+2],[start,start+2,start+3]]
        obj(name+'_wear','mesh',pose(x,y,yaw=heading),'drum_scuff',False,'barrel',
            vertices=vertices,triangles=triangles)
        if appearance.random() < .65:
            # Lumpy, flattened cloth sacks are observed on drums in the
            # organizer's historical walkthrough. Visual only; no lidar wall.
            vertices, triangles = [[0,0,-.06]], []
            rings, segments = 7, 12
            for ring in range(1,rings):
                latitude = -math.pi/2 + math.pi*ring/rings
                for step in range(segments):
                    a = math.tau*step/segments
                    radial = max(0,math.cos(latitude))**.65
                    jitter = 1+.07*math.sin(3*a+ring)
                    sx = math.copysign(abs(math.cos(a))**.48,math.cos(a))
                    sy = math.copysign(abs(math.sin(a))**.48,math.sin(a))
                    vertices.append([.19*sx*radial*jitter, .105*sy*radial*jitter,
                                     .06*math.sin(latitude)+.007*math.cos(2*a)*radial])
            for step in range(segments):
                triangles.append([0,1+(step+1)%segments,1+step])
            for ring in range(rings-2):
                for step in range(segments):
                    a=1+ring*segments+step; b=1+ring*segments+(step+1)%segments
                    c=b+segments; d=a+segments
                    triangles += [[a,b,c],[a,c,d]]
            top=len(vertices); vertices.append([0,0,.06])
            last=1+(rings-2)*segments
            for step in range(segments):
                triangles.append([last+step,last+(step+1)%segments,top])
            obj(name+'_sandbag','mesh',pose(x,y,.95,heading+appearance.uniform(0,math.tau)),
                'sandbag',False,'barrel',vertices=vertices,triangles=triangles)

    # Four GPS targets use the official order: entrance, interior pair, exit.
    nml_local=[[-15,12],[-15,1.5],[-15,-2],[-15,-12]]
    gps_points=[world(x,y) for x,y in nml_local]
    for j,y in enumerate(range(-12,13,3)):
        for side in [-1,1]:
            x,yw=physical_offset(world(-15,y),side*3.2,0)
            barrel(f'nml_edge_{j}_{side+1}',x,yw,
                   ['orange','green','red','blue'][j%4])
    for j,y in enumerate([4,-.2,-4.2]):
        x,yw=world(-15,y)
        for k,dx in enumerate([-.6,0,.6]):
            barrel(f'nml_row_{j}_{k}',*physical_offset([x,yw],dx,0))

    # Clusters and offset drums along both curved runs leave room to pass.
    rng=random.Random(config['appearance']['seed'])
    for i in range(0,len(path),26):
        if nml_mask[i] or abs(raw[i][1]) < 8:
            continue
        p, prev = path[i], path[i-1]
        dx,dy=p[0]-prev[0],p[1]-prev[1]
        norm=math.hypot(dx,dy)
        offset=rng.choice([-1,1])*.95
        barrel(f'curve_drum_{i}',p[0]-dy/norm*offset,p[1]+dx/norm*offset)

    # White disks are simulated potholes: visual hazards, not raised lidar walls.
    for j,i in enumerate([100,235,440,525]):
        p=path[i%len(path)]
        radius=course['pothole_diameter_m']/2
        disk=[[0,0,.012]]+[[radius*math.cos(k*math.tau/32),radius*math.sin(k*math.tau/32),.012]
                           for k in range(32)]
        obj(f'pothole_{j}','mesh',pose(*p),'tape',False,'pothole',vertices=disk,
            triangles=[[0,k+1,(k+1)%32+1] for k in range(32)])

    rx,ry=world(14,0)
    width,run,plateau,height=course['lane_width_m'],course['ramp_run_m'],course['ramp_plateau_m'],course['ramp_height_m']
    ramp_vertices=[]
    for yy,zz in [(-run-plateau/2,0),(-plateau/2,height),(plateau/2,height),(run+plateau/2,0)]:
        ramp_vertices.extend([[-width/2,yy,zz],[width/2,yy,zz]])
    triangles=[]
    for i in range(0,6,2): triangles += [[i,i+1,i+3],[i,i+3,i+2]]
    # Closed sides and bottom provide a watertight static ramp collision mesh.
    ramp_vertices += [[-width/2,-run-plateau/2,-.05],[width/2,-run-plateau/2,-.05],
                      [-width/2,run+plateau/2,-.05],[width/2,run+plateau/2,-.05]]
    triangles += [[0,2,4],[0,4,6],[0,6,10],[0,10,8],
                  [1,5,3],[1,7,5],[1,11,7],[1,9,11],
                  [0,8,9],[0,9,1],[6,7,11],[6,11,10],[8,10,11],[8,11,9]]
    obj('start_finish_ramp','mesh',pose(rx,ry,yaw=heading), 'ramp',True,'ramp',
        vertices=ramp_vertices,triangles=triangles,
        uvs=[[v[0],v[1]] for v in ramp_vertices])
    for label,yy in [('north_start',3),('south_start',-3),
                     ('speed_gate',-3-course['minimum_speed_check_m'])]:
        ribbon(label,[physical_offset([rx,ry],-width/2,yy),physical_offset([rx,ry],width/2,yy)],
               .15,'tape',role='gate')

    # Tree positions are manually read from the 2025 aerial; crown shape and
    # height remain procedural approximations of summer vegetation.
    features = json.loads((HERE/'reference/p37_site_features.json').read_text())
    for j,(x,y) in enumerate(item['position_enu_m'] for item in features['trees']):
        obj(f'tree_trunk_{j}','cylinder',pose(x,y,2),'bark',True,radius=.22,length=4)
        appearance=appearance_rng(f'tree_{j}')
        branch_vertices, branch_triangles = [], []
        leaf_vertices=[[] for _ in range(6)]
        leaf_triangles=[[] for _ in range(6)]

        def limb(start,end,bottom_radius,top_radius):
            axis=[b-a for a,b in zip(start,end)]
            norm=math.sqrt(sum(a*a for a in axis)); axis=[a/norm for a in axis]
            reference=[0,0,1] if abs(axis[2]) < .95 else [1,0,0]
            u=[axis[1]*reference[2]-axis[2]*reference[1],
               axis[2]*reference[0]-axis[0]*reference[2],
               axis[0]*reference[1]-axis[1]*reference[0]]
            norm=math.sqrt(sum(a*a for a in u));u=[a/norm for a in u]
            v=[axis[1]*u[2]-axis[2]*u[1],axis[2]*u[0]-axis[0]*u[2],
               axis[0]*u[1]-axis[1]*u[0]]
            base=len(branch_vertices); sides=8
            for point,radius in [(start,bottom_radius),(end,top_radius)]:
                for side in range(sides):
                    a=math.tau*side/sides
                    branch_vertices.append([point[k]+radius*(math.cos(a)*u[k]+math.sin(a)*v[k])
                                            for k in range(3)])
            for side in range(sides):
                a=base+side; b=base+(side+1)%sides
                branch_triangles.extend([[a,b,b+sides],[a,b+sides,a+sides]])

        # Tapered scaffold limbs plus individually folded, double-sided leaves
        # give an irregular, porous crown instead of intersecting green balls.
        # Each tree uses about 2,240 foliage triangles and seven draw objects.
        limb([0,0,0],[.08,-.03,4.4],.25,.10)
        phase=appearance.uniform(0,math.tau)
        for cluster in range(8):
            a=phase+cluster*math.tau/7
            reach=appearance.uniform(1.15,2.1)
            crown=[reach*math.cos(a),reach*math.sin(a),
                   4.2+(cluster%3)*.62+appearance.uniform(-.25,.25)]
            fork=[crown[0]*.55,crown[1]*.55,crown[2]-.65]
            limb([0,0,2.0+(cluster%4)*.4],fork,.10,.045)
            limb(fork,crown,.045,.012)
            for twig in range(4):
                target=[crown[0]+appearance.uniform(-.75,.75),
                        crown[1]+appearance.uniform(-.75,.75),
                        crown[2]+appearance.uniform(-.3,.65)]
                limb(fork,target,.023,.004)
            for leaf in range(35):
                azimuth=appearance.uniform(0,math.tau)
                vertical=appearance.uniform(-1,1)
                radius=appearance.random()**(1/3)
                horizontal=math.sqrt(1-vertical*vertical)
                location=[crown[0]+1.3*radius*horizontal*math.cos(azimuth),
                          crown[1]+1.1*radius*horizontal*math.sin(azimuth),
                          crown[2]+1.05*radius*vertical]
                azimuth=appearance.uniform(0,math.tau)
                tilt=appearance.uniform(-.9,.9)
                u=[math.cos(azimuth)*math.cos(tilt),math.sin(azimuth)*math.cos(tilt),math.sin(tilt)]
                v=[-math.sin(azimuth),math.cos(azimuth),0]
                n=[-math.cos(azimuth)*math.sin(tilt),-math.sin(azimuth)*math.sin(tilt),math.cos(tilt)]
                span=appearance.uniform(.24,.40); width=span*appearance.uniform(.47,.67)
                variant=appearance.randrange(6)
                vertices=leaf_vertices[variant]; triangles=leaf_triangles[variant]
                base=len(vertices)
                for along,across,lift in [(-span/2,0,0),(0,-width/2,0),
                                          (span/2,0,span*.08),(0,width/2,0),(0,0,span*.12)]:
                    vertices.append([location[k]+along*u[k]+across*v[k]+lift*n[k]
                                     for k in range(3)])
                for side in range(4):
                    a=base+side; b=base+(side+1)%4; c=base+4
                    triangles.extend([[a,b,c],[c,b,a]])
        obj(f'tree_branches_{j}','mesh',pose(x,y),'bark',False,
            vertices=branch_vertices,triangles=branch_triangles)
        for variant in range(6):
            obj(f'tree_foliage_{j}_{variant}','mesh',pose(x,y),f'leaf_{variant}',False,
                vertices=leaf_vertices[variant],triangles=leaf_triangles[variant])
    pole_positions = [item['position_enu_m'] for item in features['light_poles']]
    for j,(x,y) in enumerate(pole_positions):
        obj(f'lightpole_{j}','cylinder',pose(x,y,4.5),'metal',True,radius=.085,length=9)
        obj(f'lightbase_{j}','cylinder',pose(x,y,.35),'concrete',True,radius=.28,length=.7)
        obj(f'lighthead_{j}','box',pose(x,y,9),'metal',False,size=[1.2,.5,.15])
    # Southeast utility building: footprint read from the aerial, height assumed.
    a,b = [features['buildings'][0]['footprint_enu_m'][i] for i in (0,2)]
    bx,by = (a[0]+b[0])/2,(a[1]+b[1])/2
    obj('southeast_utility_building','box',pose(bx,by,2.8),'concrete',True,
        size=[abs(b[0]-a[0]),abs(b[1]-a[1]),5.6])
    obj('southeast_utility_roof','box',pose(bx,by,5.65),'rubber',False,
        size=[abs(b[0]-a[0])+.4,abs(b[1]-a[1])+.4,.1])
    for j,x in enumerate([105,113,121]):
        for dx in [-2,2]:
            for dy in [-2,2]:
                obj(f'pit_leg_{j}_{dx}_{dy}','cylinder',pose(x+dx,23+dy,1.4),
                    'metal',True,radius=.035,length=2.8)
        obj(f'pit_canopy_{j}','box',pose(x,23,2.85),'canopy',False,size=[4.3,4.3,.15])
        obj(f'pit_table_{j}','box',pose(x,23,.75),'concrete',True,size=[1.8,.75,.08])

    site.conform(objects)
    spawn_point=world(-15,13.1)
    spawn=[*spawn_point,site.surface_height(*spawn_point)+.1,0,0,-math.pi/2+heading]
    bounds=[min(p[0] for p in path)-half,min(p[1] for p in path)-half,
            max(p[0] for p in path)+half,max(p[1] for p in path)+half]
    prop_points=[]
    for item in objects:
        if item['role'] not in ['lane','barrel','pothole','gate','ramp']:
            continue
        x,y=item['pose'][:2]
        if item['type']=='mesh':
            yaw=item['pose'][5]; c,s=math.cos(yaw),math.sin(yaw)
            prop_points += [[x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]]
                            for p in item['vertices']]
        elif item['type']=='cylinder':
            r=item['radius']
            prop_points += [[x+dx*r,y+dy*r] for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]]
    if not all(point_in_polygon(p,pavement) for p in prop_points):
        raise ValueError('Course props extend outside the mapped P37 asphalt')
    props_bounds=[min(p[0] for p in prop_points),min(p[1] for p in prop_points),
                  max(p[0] for p in prop_points),max(p[1] for p in prop_points)]
    return dict(schema_version=1,name=config['name'],datum=datum,
                provenance='reference/sources.json',
                placement=config['placement'],
                measurement_status='2026-reference-aligned 2027 schematic; unsurveyed 2027 targets',
                environment=dict(sky_texture='reference/kloofendal_48d_partly_cloudy_puresky_1k.hdr'),
                terrain=dict(source='reference/p37_elevation_grid.json', vertical_datum='relative NAVD88',
                             reference_elevation_m=site.dem['reference_elevation_m'],
                             height_range_m=[min(v[2] for v in objects[0]['vertices']),
                                             max(v[2] for v in objects[0]['vertices'])]),
                materials=materials,objects=objects,spawn=spawn,
                cameras=[dict(id='Overview',position=[center[0]+45,center[1]-50,55],
                              target=[*center,site.surface_height(*center)]),
                         dict(id='CourseEye',position=[spawn[0]+2,spawn[1]+4,spawn[2]+1.4],
                              target=[spawn[0],spawn[1]-8,site.surface_height(spawn[0],spawn[1]-8)+.4])],
                course=dict(centerline=path,centerline_length_m=length(path,True),
                            lane_width_m=course['lane_width_m'],tape_width_m=course['tape_width_m'],
                            bounds_enu=bounds,nominal_rules_envelope_m=[36.576,30.48],
                            actual_envelope_m=[bounds[2]-bounds[0],bounds[3]-bounds[1]],
                            full_props_bounds_enu=props_bounds,
                            ramp_grade=height/run,nml_waypoints_enu=gps_points,
                            barrel_centers_enu=barrel_centers,parking_boundary_enu=pavement))


def write_waypoints(scene, filename, points):
    converter=geographic_transform(scene['datum'])
    site=SiteModel(HERE/'reference',converter)
    site.set_surface(next(o for o in scene['objects'] if o['role']=='ground'))
    from pyproj.enums import TransformDirection
    waypoints=[]
    for i,p in enumerate(points):
        next_point=points[min(i+1,len(points)-1)]
        if next_point==p and i: next_point,p_before=p,points[i-1]
        else: p_before=p
        yaw=math.atan2(next_point[1]-p_before[1],next_point[0]-p_before[0])
        lon,lat,alt=converter.transform(*p,site.surface_height(*p),direction=TransformDirection.INVERSE)
        waypoints.append(dict(latitude=round(lat,12),longitude=round(lon,12),
                              altitude=round(alt,3),yaw=yaw))
    filename.write_text('# Estimated simulation targets inside Oakland University P37.\n'
                        '# Not official or surveyed 2027 competition waypoints.\n'
                        '# Generated from warrior_simulation/igvc2027/course.yaml.\n'
                        +yaml.safe_dump(dict(waypoints=waypoints),sort_keys=False))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=HERE/'course.yaml')
    args=parser.parse_args()
    config=yaml.safe_load(args.config.read_text())
    boundary=json.loads((HERE/'reference'/'p37_boundary.geojson').read_text())
    scene=build(config,boundary['geometry'])
    (HERE/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    nav=ROOT/'warrior_navigation'/'config'
    profile = dict(world_name='igvc2027_autonav.world',
                   spawn_x=scene['spawn'][0], spawn_y=scene['spawn'][1],
                   spawn_z=scene['spawn'][2], spawn_yaw=scene['spawn'][5],
                   waypoint_file='igvc2027_autonav_nml_waypoints.yaml')
    (nav/'igvc2027_autonav_sim.yaml').write_text(
        '# Generated from warrior_simulation/igvc2027/course.yaml.\n'
        + yaml.safe_dump(profile, sort_keys=False))
    write_waypoints(scene,nav/'igvc2027_autonav_nml_waypoints.yaml',scene['course']['nml_waypoints_enu'])
    # Exercise an actual GPS leg beyond the 15 m lidar range without changing
    # the four-marker course mission: traverse directly from entrance to exit.
    markers=scene['course']['nml_waypoints_enu']
    write_waypoints(scene,nav/'igvc2027_autonav_long_leg.yaml',[markers[0],markers[-1]])
    # Reference-only full lap for testing; this is not a replacement for lane perception.
    path=scene['course']['centerline']
    write_waypoints(scene,nav/'igvc2027_autonav_reference_lap.yaml',path[::24]+[path[0]])
    print(json.dumps({key:scene['course'][key] for key in
                      ['centerline_length_m','actual_envelope_m','ramp_grade']},indent=2))


if __name__=='__main__':
    main()
