# mower_utils tests

## coverage_paths.test

Rostest that drives the real production C++ pipeline
(`mower_logic` → `MowingBehavior::create_mowing_plan`
→ `slic3r_coverage_planner::PlanPath`) for every mowing area in a fixture map,
captures every `mbf_msgs/ExePath` goal `mower_logic` would dispatch to the
controller, and asserts each pose lies in a free cell of the
`nav_msgs/OccupancyGrid` that `mower_map_service` publishes on
`/mower_map_service/map` (the same grid `move_base_flex` consumes).

No real `move_base_flex`, no FTC, no simulator drive loop. Fake action
servers hosted inside the test executable capture goals and immediately
succeed; stub services and heartbeat publishers unblock `mower_logic`'s
startup waits.

### Run

```bash
cd ~/catkin_ws
catkin_make tests
source devel/setup.bash
rostest mower_utils coverage_paths.test            # results in build/test_results/
rostest mower_utils coverage_paths.test --text     # also stream logs to stdout
```

Wall time ≈ 4 min for the 9-area sample fixture. Failures are listed per
area with the first bad pose, the worst cell value (0 = free, 100 = lethal),
and the path index within that area:

```
area[6]: 15788/15789 poses in obstacle (first at path[0] pose[0] = (-17.06, -13.50), worst cell=100, paths_in_area=14)
```

### Using a different map

Drop the map JSON into `fixtures/` and point the launch at it:

```bash
rostest mower_utils coverage_paths.test \
    fixture_path:=$(rospack find mower_utils)/test/fixtures/my_map.json
```

The map is fed to `mower_map_service` via the same `map.replace` xbot_rpc
method the web UI uses, so any map JSON the UI can import will work here.

Other overridable params (defaults in
[`coverage_paths.test`](coverage_paths.test)):

| param | default | notes |
|---|---|---|
| `fixture_path` | fixture above | map JSON to inject |
| `lethal_threshold` | 50 | 0..100 on the OccupancyGrid; poses in cells `>=` this fail. Raise if boundary-blur causes false positives. |
| `per_area_timeout_s` | 60 | if no area change in this long, test publishes `skip_area` to unstick |
| `rpc_wait_s` | 30 | timeout for the `map.replace` round-trip |

### Manual visual check

After a failing run, visualize one suspect area against the same costmap
the test used:

```bash
roslaunch mower_utils planner_test.launch AREA_INDEX:=6
```

The planned path is published on `/mower_logic/mowing_path` and the
OccupancyGrid on `/mower_map_service/map`; add both in RViz.
