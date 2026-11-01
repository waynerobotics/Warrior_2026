"""Independent checks for coordinate encoding and the supplied survey rasters."""

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from pyproj import Geod, Transformer
import pytest


REFERENCE = Path(__file__).resolve().parents[1] / "reference"


def test_historical_coordinate_encoding_matches_geographic_distance():
    data = json.loads((REFERENCE / "igvc2026_waypoints_reference.json").read_text())
    assert data["event_year"] == 2026
    points = {p["id"]: p for p in data["points"]}
    assert len(points) == 7
    north, south = points["5001"], points["5000"]
    # These are independent readings of the organizer's printed DMS digits.
    assert north["latitude"] == pytest.approx(42 + 40 / 60 + 5.771 / 3600, abs=1e-11)
    assert north["longitude"] == pytest.approx(-(83 + 13 / 60 + 9.62509 / 3600), abs=1e-11)
    assert south["latitude"] == pytest.approx(42 + 40 / 60 + 4.53974 / 3600, abs=1e-11)
    assert south["longitude"] == pytest.approx(-(83 + 13 / 60 + 9.57951 / 3600), abs=1e-11)
    azimuth, _, distance = Geod(ellps="WGS84").inv(
        south["longitude"], south["latitude"], north["longitude"], north["latitude"])
    assert distance == pytest.approx(38.00753, abs=0.001)
    assert azimuth == pytest.approx(-1.56487, abs=0.001)
    assert all(42.6678 < p["latitude"] < 42.6685 and
               -83.2197 < p["longitude"] < -83.2160 for p in points.values())
    assert data["main_axis_registration"]["rotation_from_enu_north_ccw_deg"] == pytest.approx(
        -azimuth, abs=0.001)


def test_elevation_grid_preserves_raw_navd88_values_and_pixel_origin():
    grid = json.loads((REFERENCE / "p37_elevation_grid.json").read_text())
    values = np.array(grid["elevations_m"])
    raw = np.asarray(Image.open(REFERENCE / grid["raw_geotiff"]))
    assert values.shape == raw.shape == (grid["height"], grid["width"])
    assert np.isfinite(values).all()
    assert np.max(abs(values - raw)) < 0.000051
    assert grid["native_resolution_m"] == 1
    assert grid["vertical_datum"] == "NAVD88"
    assert 260 < values.min() < values.max() < 280
    datum = grid["reference_location"]
    x, y = Transformer.from_crs(4326, grid["crs"], always_xy=True).transform(
        datum["longitude"], datum["latitude"])
    extent = grid["extent"]
    column = (x - extent["xmin"]) / (extent["xmax"] - extent["xmin"]) * grid["width"] - 0.5
    row = (extent["ymax"] - y) / (extent["ymax"] - extent["ymin"]) * grid["height"] - 0.5
    i, j = math.floor(row), math.floor(column)
    dx, dy = column - j, row - i
    interpolated = ((1 - dy) * ((1 - dx) * raw[i, j] + dx * raw[i, j + 1]) +
                    dy * ((1 - dx) * raw[i + 1, j] + dx * raw[i + 1, j + 1]))
    assert interpolated == pytest.approx(grid["reference_elevation_m"], abs=0.0001)
    assert abs(grid["reference_elevation_m"] - datum["elevation"]) > 10, (
        "NAVD88 orthometric values must not silently replace the ellipsoid datum")


def test_orthophoto_extent_and_pixels_agree():
    metadata = json.loads((REFERENCE / "p37_oakland_2025_georeference.json").read_text())
    with Image.open(REFERENCE / metadata["asset"]) as image:
        assert image.size == (metadata["width"], metadata["height"])
        assert image.getextrema()[0][0] < image.getextrema()[0][1], "Empty map response"
    assert metadata["native_resolution_m"] == pytest.approx(0.0762)
    extent = metadata["extent"]
    lon, lat = Transformer.from_crs(metadata["crs"], 4326, always_xy=True).transform(
        (extent["xmin"] + extent["xmax"]) / 2, (extent["ymin"] + extent["ymax"]) / 2)
    assert 42.6678 < lat < 42.6685 and -83.2197 < lon < -83.2160
