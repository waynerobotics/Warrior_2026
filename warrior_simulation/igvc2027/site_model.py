"""Georeferenced aerial texture and relative USGS bare-earth terrain.

The DEM is NAVD88, while the existing robot datum has an arbitrary ellipsoid
height. Only DEM height differences are used; no geoid conversion is implied.
"""
import json
import math
from pathlib import Path

from pyproj import Transformer
from pyproj.enums import TransformDirection


class SiteModel:
    def __init__(self, reference, converter):
        self.reference = Path(reference)
        self.converter = converter
        self.datum_height = converter.transform(0, 0, 0, direction=TransformDirection.INVERSE)[2]
        self.dem = json.loads((self.reference / 'p37_elevation_grid.json').read_text())
        self.aerial = json.loads((self.reference / 'p37_oakland_2025_georeference.json').read_text())
        self.mercator = Transformer.from_crs('EPSG:4326', 'EPSG:3857', always_xy=True)
        self.surface_cells = {}

    def set_surface(self, mesh):
        """Index the exported collision triangles for accurate prop placement."""
        self.surface_cells = {}
        vertices = mesh['vertices']
        for triangle in mesh['triangles']:
            points = [vertices[i] for i in triangle]
            low = [math.floor(min(p[k] for p in points)/3) for k in range(2)]
            high = [math.floor(max(p[k] for p in points)/3) for k in range(2)]
            for row in range(low[1], high[1]+1):
                for col in range(low[0], high[0]+1):
                    self.surface_cells.setdefault((col,row), []).append(points)

    def surface_height(self, x, y):
        """Interpolate the actual mesh, with raw DEM fallback outside the lot."""
        for a,b,c in self.surface_cells.get((math.floor(x/3),math.floor(y/3)), []):
            denominator = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(denominator) < 1.e-12:
                continue
            u = ((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/denominator
            v = ((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/denominator
            w = 1-u-v
            if min(u,v,w) >= -1.e-8:
                return u*a[2]+v*b[2]+w*c[2]
        return self.height(x,y)

    def projected(self, x, y):
        lon, lat, _ = self.converter.transform(x, y, 0, direction=TransformDirection.INVERSE)
        return self.mercator.transform(lon, lat)

    def enu(self, x, y):
        lon, lat = self.mercator.transform(x, y, direction=TransformDirection.INVERSE)
        # Height does not affect horizontal placement materially at this extent.
        return list(self.converter.transform(lon, lat, self.datum_height)[:2])

    def uv(self, x, y):
        east, north = self.projected(x, y)
        e = self.aerial['extent']
        return [(east-e['xmin'])/(e['xmax']-e['xmin']),
                (north-e['ymin'])/(e['ymax']-e['ymin'])]

    def pixel_enu(self, x, y, width=1800, height=800):
        """Digitized feature positions in a scaled version of the aerial."""
        e = self.aerial['extent']
        return self.enu(e['xmin']+x/width*(e['xmax']-e['xmin']),
                        e['ymax']-y/height*(e['ymax']-e['ymin']))

    def height(self, x, y):
        east, north = self.projected(x, y)
        e, w, h = self.dem['extent'], self.dem['width'], self.dem['height']
        # Raster extent describes outside edges; values are at pixel centers.
        u = max(0, min(w-1, (east-e['xmin'])/(e['xmax']-e['xmin'])*w-.5))
        v = max(0, min(h-1, (e['ymax']-north)/(e['ymax']-e['ymin'])*h-.5))
        col, row = min(w-2, int(u)), min(h-2, int(v))
        dx, dy = u-col, v-row
        a = self.dem['elevations_m']
        z = ((1-dy)*((1-dx)*a[row][col]+dx*a[row][col+1])
             + dy*((1-dx)*a[row+1][col]+dx*a[row+1][col+1]))
        return z-self.dem['reference_elevation_m']

    def mesh(self, points, triangles, max_edge=3.0, offset=0):
        """A bounded uniform grid preserves the original concave lot boundary."""
        # Clip each original polygon triangle to a uniform grid. Refining long,
        # skinny ear-clipping triangles recursively creates tens of thousands of
        # unnecessary collision triangles. Grid clipping retains the boundary
        # with bounded edge lengths and far fewer broad-phase contact candidates.
        spacing = max_edge/math.sqrt(2)
        source = [list(p[:2]) for p in points]
        vertices, faces, indices = [], [], {}

        def clip(polygon, axis, bound, keep_greater):
            output = []
            for a,b in zip(polygon, polygon[1:]+polygon[:1]):
                ina = a[axis] >= bound if keep_greater else a[axis] <= bound
                inb = b[axis] >= bound if keep_greater else b[axis] <= bound
                if ina:
                    output.append(a)
                if ina != inb:
                    t = (bound-a[axis])/(b[axis]-a[axis])
                    point = [a[k]+t*(b[k]-a[k]) for k in range(2)]
                    point[axis] = bound
                    output.append(point)
            return output

        def vertex(point):
            key = tuple(round(value, 8) for value in point)
            if key not in indices:
                indices[key] = len(vertices)
                vertices.append(list(point))
            return indices[key]

        for tri in triangles:
            polygon = [source[i] for i in tri]
            low = [math.floor(min(p[k] for p in polygon)/spacing) for k in range(2)]
            high = [math.floor(max(p[k] for p in polygon)/spacing) for k in range(2)]
            for row in range(low[1],high[1]+1):
                for col in range(low[0],high[0]+1):
                    cell = polygon
                    for axis, bound, greater in [(0,col*spacing,True),(0,(col+1)*spacing,False),
                                                  (1,row*spacing,True),(1,(row+1)*spacing,False)]:
                        cell = clip(cell,axis,bound,greater)
                        if len(cell)<3:
                            break
                    if len(cell)<3:
                        continue
                    ids = []
                    for point in cell:
                        index = vertex(point)
                        if not ids or ids[-1]!=index:
                            ids.append(index)
                    if len(ids)>1 and ids[-1]==ids[0]:
                        ids.pop()
                    for i in range(1,len(ids)-1):
                        a,b,c = [vertices[j] for j in (ids[0],ids[i],ids[i+1])]
                        area = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
                        if area>1.e-12:
                            faces.append([ids[0],ids[i],ids[i+1]])
        return dict(vertices=[[x,y,self.height(x,y)+offset] for x,y in vertices],
                    triangles=faces, uvs=[self.uv(x,y) for x,y in vertices])

    def surround(self, pavement, pavement_triangles):
        e = self.aerial['extent']
        corners = [self.enu(e[x], e[y]) for x,y in
                   [('xmin','ymin'), ('xmax','ymin'), ('xmax','ymax'), ('xmin','ymax')]]
        # Cut the parking surface out, rather than stacking two interpolated
        # DEMs: different mesh densities can otherwise create hidden bumps.
        background = self.mesh(corners, [[0,1,2],[0,2,3]], max_edge=8)
        cutters = [[pavement[i] for i in t] for t in pavement_triangles]

        def clip(polygon, a, b, inside):
            def side(p):
                return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
            result = []
            for p,q in zip(polygon,polygon[1:]+polygon[:1]):
                sp,sq = side(p),side(q)
                pin,qin = (sp>=0,sq>=0) if inside else (sp<=0,sq<=0)
                if pin:
                    result.append(p)
                if pin != qin:
                    t = sp/(sp-sq)
                    result.append([p[k]+t*(q[k]-p[k]) for k in range(2)])
            return result

        def bounds(p):
            return min(v[0] for v in p),min(v[1] for v in p),max(v[0] for v in p),max(v[1] for v in p)

        cutter_bounds = [bounds(c) for c in cutters]
        vertices,indices,faces = [],{},[]

        def vertex(p):
            key = tuple(round(v,8) for v in p[:2])
            if key not in indices:
                indices[key] = len(vertices)
                vertices.append(list(p[:2]))
            return indices[key]

        for tri in background['triangles']:
            polygon = [background['vertices'][i][:2] for i in tri]
            bbox = bounds(polygon)
            pieces = [polygon]
            for cutter,cb in zip(cutters,cutter_bounds):
                if bbox[0]>=cb[2] or bbox[2]<=cb[0] or bbox[1]>=cb[3] or bbox[3]<=cb[1]:
                    continue
                remaining = []
                for piece in pieces:
                    inside = piece
                    for a,b in zip(cutter,cutter[1:]+cutter[:1]):
                        outside = clip(inside,a,b,False)
                        if len(outside)>=3:
                            remaining.append(outside)
                        inside = clip(inside,a,b,True)
                        if len(inside)<3:
                            break
                pieces = remaining
            for piece in pieces:
                ids = [vertex(p) for p in piece]
                for i in range(1,len(ids)-1):
                    a,b,c = [vertices[j] for j in (ids[0],ids[i],ids[i+1])]
                    area = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
                    if area>1.e-9:
                        faces.append([ids[0],ids[i],ids[i+1]])
        # Boolean clipping may split one side of a shared edge. Insert those
        # vertices in its neighbor too, so the collision surface has no cracks.
        bins = {}
        for i,p in enumerate(vertices):
            bins.setdefault((math.floor(p[0]/4),math.floor(p[1]/4)),[]).append(i)
        conformed = []
        for face in faces:
            outline = []
            for a,b in zip(face,face[1:]+face[:1]):
                p,q = vertices[a],vertices[b]
                dx,dy = q[0]-p[0],q[1]-p[1]
                norm2 = dx*dx+dy*dy
                found = [(0.,a)]
                for row in range(math.floor(min(p[1],q[1])/4),math.floor(max(p[1],q[1])/4)+1):
                    for col in range(math.floor(min(p[0],q[0])/4),math.floor(max(p[0],q[0])/4)+1):
                        for j in bins.get((col,row),[]):
                            r = vertices[j]
                            t = ((r[0]-p[0])*dx+(r[1]-p[1])*dy)/norm2
                            cross = abs((r[0]-p[0])*dy-(r[1]-p[1])*dx)
                            if 1.e-8<t<1-1.e-8 and cross<1.e-8*math.sqrt(norm2):
                                found.append((t,j))
                outline.extend(j for _,j in sorted(found))
            if len(outline)==3:
                conformed.append(outline)
            else:
                center = vertex([sum(vertices[i][k] for i in face)/3 for k in range(2)])
                conformed.extend([[a,b,center] for a,b in zip(outline,outline[1:]+outline[:1])])
        return dict(vertices=[[x,y,self.surface_height(x,y)] for x,y in vertices],
                    triangles=conformed,uvs=[self.uv(x,y) for x,y in vertices])

    def conform(self, objects):
        """Place props on measured terrain; deform tape/ramp to stay attached."""
        for item in objects:
            if item['role'] in ('ground', 'terrain'):
                continue
            x, y = item['pose'][:2]
            base = self.surface_height(x, y)
            if item['type'] == 'mesh' and item['role'] in ('lane', 'gate', 'ramp', 'curb', 'pothole'):
                c, s = math.cos(item['pose'][5]), math.sin(item['pose'][5])
                for p in item['vertices']:
                    p[2] += self.surface_height(x+c*p[0]-s*p[1], y+s*p[0]+c*p[1])-base
            item['pose'][2] += base


def curb_mesh(a, b, width=.18, height=.12):
    """A closed curb sampled along its length for subsequent terrain draping."""
    distance = math.dist(a,b)
    count = max(1, math.ceil(distance/2))
    nx, ny = -(b[1]-a[1])/distance*width/2, (b[0]-a[0])/distance*width/2
    vertices, triangles = [], []
    for i in range(count+1):
        x,y = [a[k]+(b[k]-a[k])*i/count for k in range(2)]
        vertices.extend([[x+nx,y+ny,-.08], [x-nx,y-ny,-.08],
                         [x-nx,y-ny,height], [x+nx,y+ny,height]])
    for i in range(count):
        for j in range(4):
            a0,b0 = 4*i+j, 4*i+(j+1)%4
            triangles.extend([[a0,b0,b0+4], [a0,b0+4,a0+4]])
    last = 4*count
    triangles.extend([[0,2,1],[0,3,2], [last,last+1,last+2],[last,last+2,last+3]])
    return dict(vertices=vertices, triangles=[list(reversed(t)) for t in triangles])
