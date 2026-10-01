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

## The 3D representations: sparse_voxel, multi_level_surface
`map.type: sparse_voxel` or `multi_level_surface` (launch `map:=`, or `perception_map:=` of
sac_bringup; `_memoryless` for the mode below) keep the sources' obstacle points in 3D
instead of each source's 2D layer, and decide per column, over time, what the car cannot
pass (`traversability.hpp`, the same rules for both):
- the column's ground: the lowest level seen there (the ground map keeps a height more than
  `level_gap` (1 m) over the one it holds as another level: a deck does not replace the road
  under it), else a neighbour's within 1 m, else the ground filter's estimate under the point
- an obstacle between ground + `min_obstacle_height` (0.25 m) and the car's height +
  `clearance` (2.13 m): what is over the car is kept but not in the way, and a ceiling lower
  than the car closes the way (a tunnel 1.8 m high is closed, one 3 m high is not: unit test)
- `max_step` (off): the ground rising or falling more than this to the next cell
- each element keeps its lowest and highest point, so the band's edges are those of the points
Both are stored sparse in height and dense over the ground (`column_map.hpp`: each cell of the
window holds the few elements of its column), so a ray is traced in 2D as before and lowers
the elements it passes at the height it runs there. They differ in the elements:
- `sparse_voxel`: voxels of 0.2 m; a point joins the voxel it falls in
- `multi_level_surface`: height intervals, after Triebel, Pfaff and Burgard's multi-level
  surface maps (IROS 2006); a point within `merge_gap` (0.3 m) of an interval joins it, and
  intervals that come that close merge: a pole or a pedestrian is one interval, a deck
  another 8 m up; at most `max_levels` (6) per cell. An interval only grows while it lasts
  (a ray lowers it as a whole, taking it 0.1 m thicker than its points) The roof lidar's chain has no height_band for it: the map keeps
what is over the car too.

Two modes (`map.memory`):
- `true`: what was seen stays until rays pass through it or it fades
- `false`: each source's voxels are its last scan (or its scans of the last `window` [s]),
  rebuilt with every scan, no rays traced

`/sac/perception/map` (`publish_map`) shows what a representation holds, for any of them:
voxels (or direct_projection's occupied cells at their ground), with `blocks` 1 for what is in
the grid and 0 for what is kept but not in the way.

Without the simulation (unit test Benchmark, the same VLP-16 scan), into the map and ground
map, then decaying and projecting:

| Map | Insert | Decay and project |
|---|---|---|
| direct_projection | 1.8 ms | 0.4 ms |
| sparse_voxel, memory / no memory | 4.7 / 1.0 ms | 1.3 / 1.2 ms |
| multi_level_surface, memory / no memory | 4.8 / 1.1 ms | 1.2 / 1.3 ms |

With memory the time goes into tracing every ray from the sensor (direct_projection traces
only a ray's low end), not into the elements: intervals save little over voxels. Two
shortcuts took sparse_voxel from 9.1 ms to 4.7 (6.4 with the first alone):
- empty blocks of 8 x 8 cells crossed in one step, as OpenVDB's hierarchical DDA: the same
  elements lowered (unit test); free and unknown are told apart per block there
- `merge_rays`: the rays ending in the same cell and voxel height traced once, to their mean
  end, as OctoMap's discretized insertion and Voxblox's merged integrator (the points still
  go in one by one). Near the car dozens of ground returns share a cell; a cell along them is
  lowered once for them, so what has gone is cleared over a few scans (within 0.3 s in the
  unit test) instead of one
Decaying and projecting go over the blocks holding elements only.

The same lap as above (obstacles beside the route, 4.4-4.6 km each):

| Map | Grids with a false obstacle in the lane | False cells per grid | Small obstacles first seen | Roof lidar per scan |
|---|---|---|---|---|
| direct_projection | 1.4 % | 0.03 | 16-23 m | 11.9 ms |
| sparse_voxel, memory | 1.1 % | 0.13 | 13-29 m | 36.8 ms |
| sparse_voxel, no memory | **0.2 %** | 0.02 | 8-16 m | 10.1 ms |
| multi_level_surface, memory | 0.9 % | 0.14 | 15-24 m | 34.6 ms |
| multi_level_surface, no memory | **0.2 %** | 0.03 | 14-16 m | 11.7 ms |
| sparse_voxel, memory, blocks and merged rays (5.2 km) | 0.9 % | 0.08 | 14-25 m | 24.2 ms |
| multi_level_surface, memory, blocks and merged rays (5.2 km) | 0.7 % | 0.12 | 17-27 m | 22.2 ms |

All ten obstacles were seen each time, their cells within 0.25 m of their footprints. Without
memory a small obstacle shows only in scans that hit it: a 16-beam lidar misses a cone
between its rings, so it is seen late (8 m); with memory it is kept, and seen from 13-29 m.
Memory costs: every ray traced in 3D, 3-4 times the time of direct_projection with Gazebo on
the same CPU. Most of the 3D maps' false cells are at one place, (-8, -117) to (7, -102),
for every one of them; not looked into yet. They show for a scan or two and are cleared.
direct_projection stays the default.

## The camera: depth_image
`camera:=true` (perception.launch.py; `perception_camera:=` of sac_bringup) adds the front
camera (`config/camera.yaml`). The simulated ZED 2 gives a 1280 x 720 depth image at 15 Hz;
its full point cloud would be 921 000 points, 11 MB a frame. The `depth_image` source takes
the depth image itself and back-projects every 4th pixel of every 4th row (58 000 points).
A camera cannot find the ground: `ground_from_map` takes each point's ground from the roof
lidar's ground map before the filters (within 0.2 m of it: road; height_band measures from
it). Where none is known within 2 m (between the lidar's far rings) a point only clears the
space before it, so a road rising where the lidar has not seen it yet is no obstacle (the
front 2D lidar's lesson). It marks up to 12 m: stereo depth errors grow with the square of
the range.

The lap with obstacles beside the route (direct_projection, 5 km): with the camera 2.6 % of
the grids had a false obstacle in the lane (0.05 cells a grid; 1.4 % and 0.03 without), the
small obstacles were first seen 14-24 m out, as without it (the lidar sees them before 12 m).
The camera is for what the lidars miss (close in front of the car, very low things), which
this lap does not have; it stays off by default. It took 25.5 ms a frame at first: each point
looked for the ground in 441 cells (2 m around) where its own cell had none. The ground map
is now searched ring by ring outwards, stopping once no nearer cell can come (the same
nearest ground: unit test against the whole square): 4.9 ms a frame, and the grid step's
peaks went from 340 to 19 ms.

## Objects: clusters followed over time
For what moves (a pedestrian crossing) the grid is not enough: the planner needs where it
will be. Every grid step (`objects` in the YAML, `objects.hpp`), the map gives the obstacles
in the car's way seen in the last 0.15 s within 30 m (`recent()`, any representation), a
clusterer groups them, a tracker follows the groups; `/sac/perception/objects`
(sac_perception_msgs/TrackedObjects: id, class, position, velocity and their covariances,
box, age) and boxes with velocity arrows on `/sac/perception/objects/markers`.
- `connected_components`: cells within 0.4 m of each other are one cluster; its box along
  its main direction (PCA), a class by its size (pedestrian: up to 1.2 m across, 1-2.2 m
  high; vehicle: 2.5-6 m long; small: lower than 0.8 m; structure: longer than 8 m)
- `kalman_tracker`: a constant-velocity Kalman filter per object, clusters joined to tracks
  by Mahalanobis distance (3 sigma, 2 m at most), nearest pairs first; confirmed after 3
  scans, dropped 0.5 s unseen. A 10 Hz lidar's scan is in two 20 Hz grids: a track takes a
  cluster only if newer than its last
- `moving`: confirmed, faster than 0.5 m/s (2 sigma over half of it), and, after Wang et
  al.'s free-space test (DATMO, IJRR 2007), taking cells seen free just before (a tenth of
  its cells or more, update after update for 0.3 s). The maps keep when each cell was last
  seen free (a ray passing it or ending on the ground there)

The first lap told why the free-space test: a barrier or a slope beside the track is cut by
one lidar ring into a short line, and as the car drives the ring moves on with it: the line
slides along the barrier at nearly the car's speed. Its centre moves, its cells were never
free (the rays end on it). Also a cluster much longer than wide is trusted across, hardly
along (its ends are where the view ends).

`many` obstacles (gazebo_environment: 34 beside the route, people alone and in pairs 1 m
apart, cones, poles, boxes, parked cars), direct_projection, a lap (4.4-4.9 km); per obstacle
within 25 m of the car:

| | Without the free-space test | With it |
|---|---|---|
| a confirmed object on it | people 93-100 %, cones 47-81 %, small boxes 45-65 % | people 92-100 %, cones 75-80 %, small boxes 46-65 % |
| people classed pedestrian | 74-95 % | 68-92 % |
| pairs 1 m apart seen as two | 99-100 % | 98-100 % |
| standing obstacles' speed, p95 | people 0.2-0.3, cars 0.8-4.0 m/s | people 0.2-0.3, cars 0.6-2.3 m/s |
| id switches (34 obstacles) | 61 | 60 |
| messages with a `moving` object | 56 % (1083 ids) | 20 % (271 ids) |

Everything in this lap stands still, so every `moving` object is false.

### People crossing, and free space with high confidence
`pedestrians:=crossing` (gazebo_environment `walk_pedestrians`: cylinders moved through
set_pose) has four people crossing 12-16 m of road, at 1.0-2.5 m/s, starting when the car is
35-40 m away; the local planner yields (its DESIGN.md). Two lessons on the way:
- "seen free in the last second" (above) came too late for one of them: 25 m out the 16-beam
  lidar sees the road in rings, not every second; widening the window let more slopes in, and
  a stop-gap rule in the planner (yield to small objects with a sure speed) made the car stop
  for them (8-15 times a lap); the objects around those stops (size, height, speed, its
  certainty, age) were not told apart from the people by any threshold
- after Dynablox (Schmid et al., RA-L 2023): free space known with high confidence, with no
  time limit. A cell is ever-free once it and its 8 neighbours were seen free after they were
  last occupied and not occupied for 0.5 s (`map.free_space.burn_in`; occupied observations
  0.2 s apart are one, `sparsity`); it stays so until occupied 1 s in a row (`static_after`:
  something came to stay). A point taking an ever-free cell, or one next to it, moved there;
  a cluster with such points, update after update for 0.3 s, is moving. Only the grid's window
  is kept (80 m, 40 m each way: a cell leaving it is forgotten, label and all): the road ahead
  comes into it 40 m out, the lidar's ground rings (34 and 20.5 m out on flat ground) sweep
  over it as the car drives, and once seen free it stays known free however long ago that
  was. "Seen free in the last second" had lost it again by 25 m. A cell never seen free yet
  (just come into the window, or in a lidar's shadow) tells nothing

| Crossing lap, planner on | "recently free" + the stop-gap rule | ever-free (Dynablox) |
|---|---|---|
| person first `moving`, car this far | 15.6-29.8 m | 21.1-29.9 m |
| closest, car's body to the person | 4.25-7.57 m | 4.36-7.52 m |
| stops with nobody within 30 m | 8-15 (10.8-21.7 s) | 2 (2.2 s) |

On the `many` lap the share of messages with a false `moving` object is about the same (37 %;
3 % within 4 m of the route): most are off the track, where the ground filter takes a ring
across a grassy slope for an obstacle on cells seen free before; the ground filter's errors on
slopes, not the free space, are left for those. What is left are
mostly thin clusters 17-32 m away, 1-2 m high: slopes and growth beside the track, whose cells
were seen free a moment before (the last scan's rays ran low over them). Poles are classed
pedestrian (the same size). A parked car changes id as the car passes it (the side it shows
changes). 91 % of the confirmed objects within 25 m lie on no spawned obstacle (terrain,
barriers): the grid stops the planner for them anyway; the objects are for what moves.
Not caught by the free-space test: a car driving away (the space it moves into is hidden
behind it); cells it leaves free would tell.

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
