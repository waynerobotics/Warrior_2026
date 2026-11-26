# warrior_costmap_plugins

Nav2 costmap layers for Warrior.

## LaneLayer

`warrior_costmap_plugins::LaneLayer` turns white lane paint detected by
[warrior_lane_detection](../warrior_perception/warrior_lane_detection/) into
costmap cost. It has two modes.

**Hard boundary (`lethal: true`).** The robot never crosses a line, but it
can get stuck: the planner fails whenever the robot centre is already inside
the band (`Starting point in lethal space`), recoveries spin and back up, and
gaps in the remembered paint become the only way through. Not used in
[nav2_gps_sim_params.yaml](../warrior_navigation/config/nav2_gps_sim_params.yaml);
see below.

- A cell on the paint is `LETHAL_OBSTACLE` (254).
- Every cell within `keepout_radius` of paint is `INSCRIBED_INFLATED_OBSTACLE`
  (253). Set it to the footprint's half-width (0.66 m for Warrior). Smac
  checks only the robot centre against 253, so this band is what keeps the
  whole body off the paint in the plan.
- Beyond the band, `lane_cost` falls linearly to 0 over `falloff_radius`,
  which keeps the robot centred.
- Regulated Pure Pursuit checks the real footprint polygon against lethal
  cells and stops before the body touches paint, even if a plan is wrong.
- The costmap `footprint` must be the real shape, not a covering circle, or
  the lane is narrower than it really is.

**High but passable (`lethal: false`, the default).** Used twice in
[nav2_gps_sim_params.yaml](../warrior_navigation/config/nav2_gps_sim_params.yaml),
so the planner always prefers driving over a pothole to crossing a line:

- `lane_layer` (lane lines, `/perception/lane_points`): `lane_cost` 252, the
  highest passable cost, out to `keepout_radius` 0.66 m (footprint
  half-width), then 0 over 0.5 m. The planner always has a solution, and a
  line is crossed only when there is no other way at all.
- `pothole_layer` (`/perception/pothole_points`, which
  [warrior_lane_detection](../warrior_perception/warrior_lane_detection/)
  splits off from the lines by blob shape): `lane_cost` 120 out to 0.6 m,
  then 0 over 0.3 m.

In detail:

- Cells within `keepout_radius` of paint get `lane_cost` (at most 252), then
  the cost falls linearly to 0 over `falloff_radius`. A plateau out to about
  the robot's half-width keeps the whole body off the pothole when there is
  room.
- The planner's `cost_travel_multiplier` decides how much detour avoiding it
  is worth. When the lane is too narrow beside a pothole (a centred IGVC
  pothole leaves 1.47 m to the line; Warrior is 1.29 m wide), the robot drives
  over it rather than crossing a line.
- Also suits courses with lines that must be driven over, such as start or
  finish lines across the lane.

- **Marking and clearing:** the layer subscribes to two flat clouds in
  `base_footprint` that share the image stamp.
  - `mark_topic` (default `/perception/lane_points`) holds the paint cells.
  - `clear_topic` (default `/perception/lane_free_points`) holds every ground
    cell the camera saw.
  
  For each stamp the layer clears first and then marks. Lidar never clears
  paint. Ground the camera cannot see (its blind zone in front of the bumper)
  is never cleared, so lines stay remembered while the robot drives over them.
- **Combining with other layers:** costs are merged with max. Lethal cells
  stay lethal. Cells marked unknown by other layers take the lane cost. Put
  the layer before `inflation_layer`.

| Parameter | Default | Meaning |
|---|---|---|
| `enabled` | `true` | |
| `mark_topic` | `/perception/lane_points` | paint cells (PointCloud2) |
| `clear_topic` | `/perception/lane_free_points` | visible ground cells (PointCloud2) |
| `lethal` | `false` | hard boundary: lethal paint plus an inscribed keep-out band |
| `keepout_radius` | 0.0 | m, inscribed band (lethal) or `lane_cost` plateau (passable); about the footprint half-width |
| `lane_cost` | 250 | soft cost, 1-252: the plateau (passable) or at the band edge (lethal) |
| `falloff_radius` | 0.9 | m, soft cost reaches 0 this far past `keepout_radius` |
| `clear_resolution` | 0.2 | m, must match `free_resolution` in lane_segmentation |
| `pair_timeout` | 0.5 | s, apply a mark/clear cloud alone if its partner is late |
| `max_age` | 2.0 | s, keep waiting for TF this long before dropping |

Tests: `colcon test --packages-select warrior_costmap_plugins`. The tests use
the Pixi environment's prebuilt GTest, because `gtest_vendor` does not compile
with its GCC.
