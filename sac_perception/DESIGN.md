# sac_perception: design

The local occupancy grid around the car, for a local planner to avoid obstacles along the
route. Any number of sensors, each set up in YAML; exchangeable pieces as plugins, like
sac_localization.

## Pieces
```
sensor topics ──► sources (a plugin per message type) ──► map representation (plugin) ──► /sac/perception/grid
                   pointcloud: filter chain (plugins)       direct_projection: a layer       (20 Hz)
                   laser_scan                               per source, combined
                   occupancy_grid
```
- **Sources** (`sources.hpp`), one per sensor in `sources.names`, by message type:
  `pointcloud` (3D lidar, depth camera), `laser_scan` (2D lidar), `occupancy_grid` (another
  node's grid, e.g. a camera's segmentation). A sensor more is an entry more in the YAML.
  Where a sensor sits comes from TF (the message's `frame_id`, the URDF's mount), or from
  `mount:` for a sensor without TF. Each message is placed with `odom -> sensor` at its own
  stamp.
- **Filters** (`filters.hpp`), a chain per point cloud source, in the YAML's order:
  `crop_box` (the car itself), `voxel` (one point per voxel), `ground_height` (flat ground),
  `ground_patchwork` (slopes, crests; below), `height_band` (obstacles between two heights
  over the ground). A new method is a new plugin and a name in the list; each source has its
  own instances, so a filter that learns (ground_patchwork) learns per sensor.
- **Map representation** (`map_representation.hpp`), `map.type` in the YAML: how what the
  sources saw is kept and turned into the 2D grid the planner reads. Every source hands it
  the same thing, a scan of rays in `odom` ending in an obstacle or free point, with the
  ground height under each. `direct_projection` is the grid below, as it was: the sources'
  filters decide per point, each source writes its own layer. 3D representations (sparse
  voxels, multi-level surfaces) come next, behind the same interface and the same output.
- **The car's box** (`vehicle.from: robot_description`): read from the URDF that
  robot_state_publisher publishes (`robot_geometry.hpp`: every visual and collision shape,
  joints at zero, STL and OBJ meshes read for their bounds). The sources start once it has
  come. Filters can follow it instead of numbers in the YAML: `crop_box from: vehicle` (the
  car itself, grown by a margin) and `height_band max_from: vehicle` (below).
- **Grid** (`grid.hpp`): a square window in `odom` (80 x 80 m, 0.2 m) that follows the car
  by whole cells and never turns. Each source writes its own layer of log-odds: returns mark
  their cells, the space before them is cleared (for a 3D lidar only where the ray runs
  lower than `clear_height` over the ground: a ray passes over low obstacles far from its
  end). Layers fade (`decay`) where nothing is seen. The published grid is the most occupied
  layer per cell. A distance transform (exact, linear time) gives each cell's distance to the
  nearest obstacle, for the planner's collision checks.

Why `odom`: fixed to `base_footprint` the grid would turn with the car and everything in it
would have to be moved every step; `map` jumps when GNSS corrects the global pose, and a scan
would no longer line up with the previous one. `odom` is smooth over seconds.

## Ground: Patchwork++
`ground_patchwork` follows Patchwork++ (Lee, Lim, Myung, IROS 2022) step by step, with its
defaults, written for this package (not the library, so it can be changed here):
RNR (reflections, when the cloud has intensity), CZM (4 concentric zones of rings and
sectors), R-VPF (vertical structures out first, zone 0), R-GPF (a plane per bin from its
lowest points, refined), GLE (upright; near the sensor also not elevated or flat, and
facing it from below), TGR (flat enough bins reverted), A-GLE (thresholds and the sensor's
height learnt from the ground seen). Added for the grid: every point's ground height.

**VLP-16.** Patchwork++'s defaults are for 64 beams. A 16-beam lidar sees the ground as a few
sparse rings (6.7, 7.8, 9.2, 11.3, 14.6, 20.5, 34 m on flat ground from the roof); slopes cut
them into short pieces per bin, and Patchwork++ does not take bins with fewer than
`num_min_pts` points as ground. With 3 instead of 10 the unit tests' VLP-16 scans (flat,
uphill, crest, car pitched 2 deg, a box 18 m ahead) find over 99 % of the ground and over
95 % of the box.

**Additions to Patchwork++** (each switchable in the YAML), both found at the bridge over
the Sonoma track, where the local planner saw no way through for 5 s:
- `enable_grade_check`: Patchwork++ checks a bin's height only in the rings of interest near
  the sensor; further out any upright plane is ground, and the flat deck 8 m over the road,
  25-30 m ahead, was: its railings then stood 1-2 m "over the ground" as obstacles, and the
  deck went into the shared ground map. A plane further than 0.5 m + 20 % of its range from
  the ground under the sensor is not ground. Each bin is held against the sensor alone, not
  against its neighbour, so an error cannot carry on outwards (the first attempt's failure).
- `lpr_ground_tolerance` 1.0 m instead of 0.5: under the bridge the road rises 0.5 m within
  a few metres, and the ground height of a rejected bin fell back to a plane from nearer
  the sensor, leaving the road 0.54 m "high": a line of obstacles across the lane.
Unit test: a deck 8 m up with 1 m railings, the VLP-16's 16 channels: no deck point is
ground or an obstacle, over 99 % of the road is ground. In the simulation the local
planner's cycles with no free candidate went from 54 in one pass under the bridge to 9 in
4.8 km, all on the approach to the bridge: the grade window grows with the range, so the
deck (8 m up) is rejected only within 37.5 m, and the planner looks 40 m ahead. A tighter
max_grade (0.15: 50 m) would also reject real ground far out where a descent turns into a
climb, so it stays 0.2.

A first attempt (a plane per bin, each checked against the nearer bin's) failed on the
VLP-16: a bin holding a single ring is a line, a plane through a line tilts freely, and the
check carried one tilted plane outwards: the flat road 20 m ahead came out 0.7 m above the
ground, a false obstacle in the lane in 97 % of the grids. Patchwork++ checks each bin
against what the whole scan's ground looks like, so one bad bin stays one bad bin.

## The height band: bottom from the ground, top from the car
`height_band` keeps the obstacle points between two heights over the ground under them. Its
bottom follows the ground the filters found (ground_patchwork), so it works on slopes. Its
top was a fixed 2.5 m; now, with `max_from: vehicle`, it is the car's height from the URDF
plus `clearance` (0.3 m): what the car would touch. In the simulation the box comes out as
x -1.35..1.44 m (the front 2D lidar), y -1.03..1.03 m, z 0..1.83 m (the top of the VLP-16), so
the band runs 0.25..2.13 m. A branch hanging down to 1.9 m is an obstacle, a sign at 4 m is
not; another car or a sensor moved higher needs no change in the YAML. `max_from: sensor`
takes the sensor's height instead, `fixed` the number.

## The shared ground map
Sources that find the ground write its height into the grid (`provides_ground`); sources that
cannot tell the ground from an obstacle read it (`ground_margin`): the front 2D lidar's plane
(0.45 m up) meets the road where it rises ahead, and a return within 0.2 m of the ground the
roof lidar saw there (nearest known cell within 1 m, seen in the last 3 s) is road, not an
obstacle. Neither names a sensor; where no ground is known a return stays an obstacle.

## Timing
Without the simulation (unit test Benchmark, a full VLP-16 scan, 28 800 points): crop_box
0.1 ms, ground_patchwork 3.2 ms, height_band 0.05 ms, into the grid (17 000 rays) 1.5 ms,
combining the layers 0.4 ms. A 0.1 m voxel first keeps 82 % of the points and costs 1.8 ms
itself, so it is not in the default chain.
Sources never wait for TF: a message whose `odom -> sensor` is not there yet is kept and
processed at the next tick (at most 50 ms later). Before, waiting up to 100 ms counted as
processing time and held up the node. In the simulation (Gazebo on the same CPU) the roof
lidar then takes 8 ms per scan on average (15 ms with two more perception nodes running for
the comparison), the front lidar 0.4 ms, a grid step 1 ms.

Each filter can report numbers about the last scan (`diagnostics()`), published with the
source's timing on `/sac/perception/timing`: ground_patchwork gives why points were not
ground (sparse bin, not upright, facing away, elevated), how many TGR reverted, and the
learnt sensor height and thresholds, for tuning on the real car.

## Results in the simulation
A lap of Sonoma (3.1 km, 10 m/s, ground truth TF) with ten obstacles 3-4 m beside the route
(`obstacles:=beside_route`: boxes, cones, a pole, cars), against their true footprints:

| Setup | Grids with a false obstacle in the lane (+-2.5 m, < 30 m) | False cells per grid | Obstacles seen |
|---|---|---|---|
| roof lidar, first ground filter | 97 % | 193 | 10 / 10 |
| roof lidar, ground_patchwork | 8.6 % | 0.5 | 10 / 10 (small ones from 10-13 m) |
| roof + front lidar | 24 % | 6.7 | 10 / 10 (from 19-30 m) |
| roof + front lidar, shared ground map | 19 % | 3.3 | 10 / 10 (from 23-30 m) |
| roof + front lidar, ground map, front marks up to 15 m | **4.4 %** | **0.24** | 10 / 10 (small ones from 13-16 m) |
| the same as `direct_projection`, self and band from the car's box (4.6 km) | **1.4 %** | **0.03** | 10 / 10 (small ones from 16-23 m) |

The occupied cells near an obstacle are all within 0.25 m of its footprint.

The last row is the move behind the map interface: `direct_projection` gives the same grid
as before for the same scans (unit test), and the band's top came down from 2.5 m to the
car's 1.83 m + 0.3 m. What is left is on the steep climb around (-100, -193). The roof lidar
took 11.9 ms per scan (8 ms before; Gazebo runs on the same CPU, and varies from run to run).

Where the false obstacles came from, logged once a second over a lap with a grid per source
and the ground filter's diagnostics: nearly all from the front 2D lidar, 14-29 m ahead,
where its plane meets a rising road; the roof lidar alone had one or two cells at a few
places, and ground_patchwork rejected no ground for elevation (the learnt sensor height
stayed 1.76-1.80 m, true 1.79). The shared ground map did not catch them: from the roof the
ground is seen in rings (20.5 and 34 m out on flat ground), and between them it is not known
yet. So the front lidar marks obstacles only up to `max_mark_range` (15 m) and beyond that
only clears the space before its returns: near the car it covers the roof lidar's blind
zone, far away the roof lidar sees for both. What is left clusters on the steep twisting
climb around (-31, -154).
