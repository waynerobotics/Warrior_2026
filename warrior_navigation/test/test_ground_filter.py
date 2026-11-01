"""Ground must disappear without erasing nearby obstacles on pitched roads."""
import numpy as np
from warrior_navigation.ground_filter import estimate_ground_plane, obstacle_mask


def test_sloping_road_and_barrels_are_separated():
    rng = np.random.default_rng(42)
    xy = rng.uniform(-5, 5, (1800, 2))
    road = np.column_stack((xy, .07*xy[:, 0] - .035*xy[:, 1] + rng.normal(0, .005, len(xy))))
    barrel_xy = np.array([[3., .5], [3.1, .5], [3., .6], [7., -2.]])
    barrel = np.column_stack((barrel_xy, .07*barrel_xy[:, 0] - .035*barrel_xy[:, 1] + [.25, .5, .8, .8]))
    cloud = np.vstack((road, barrel))
    plane = estimate_ground_plane(cloud)
    assert np.allclose(plane, [.07, -.035, 0], atol=.004)
    mask = obstacle_mask(cloud)
    assert not mask[:len(road)].any()
    assert mask[len(road):].all()


def test_curved_road_far_returns_and_near_step():
    rng = np.random.default_rng(2027)
    xy = rng.uniform(-14, 14, (12000, 2))
    z = .02*xy[:, 0] + .0015*xy[:, 0]**2
    cloud = np.column_stack((xy, z))
    # A 35 cm obstacle at 3 m remains an obstacle, unlike gently curving ground.
    cloud = np.vstack((cloud, [3., 0., .02*3 + .0015*9 + .35]))
    mask = obstacle_mask(cloud)
    assert not mask[:-1].any()
    assert mask[-1]


def test_no_ground_and_invalid_returns_use_bounded_fallback():
    cloud = np.array([[2., 0., .8], [2., 1., 1.], [np.nan, 0., 0.], [16., 0., .8]])
    assert np.allclose(estimate_ground_plane(cloud), [0, 0, 0])
    assert obstacle_mask(cloud).tolist() == [True, True, False, False]
    assert obstacle_mask(np.empty((0, 3))).size == 0


def test_base_link_offset_preserves_the_same_classification():
    x, y = np.meshgrid(np.linspace(-4, 4, 21), np.linspace(-4, 4, 21))
    road = np.column_stack((x.ravel(), y.ravel(), (.04*x).ravel()))
    cloud = np.vstack((road, [2., 0., .55]))
    shifted = cloud - [0, 0, .640773]
    assert np.array_equal(obstacle_mask(cloud), obstacle_mask(shifted, ground_height=-.640773))


def test_radial_profiles_follow_a_depression_without_erasing_barrels():
    radius, angle = np.meshgrid(np.arange(1, 15, .25), np.linspace(-np.pi, np.pi, 360, endpoint=False))
    x, y = radius*np.cos(angle), radius*np.sin(angle)
    z = .0045*radius**2
    ground = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    # An 82 cm drum remains above the estimated road even on its far upslope.
    barrels = np.array([[3., 0., .0045*9+.82], [8., 0., .0045*64+.82],
                        [12., 0., .0045*144+.82]])
    mask = obstacle_mask(np.vstack((ground, barrels)))
    assert np.count_nonzero(mask[:len(ground)]) < len(ground)*.001
    assert mask[len(ground):].all()
