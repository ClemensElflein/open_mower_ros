# OpenMower simulation stack

Runs the real `open_mower_ros` logic and `OpenMowerApp` against a simulated mower
instead of real hardware, so you can test a specific combination of versions without a
robot. `mower_logic`, navigation, the coverage planner, monitoring, etc. all run for
real, unmodified, in the `open_mower_ros` container - only the low-level board (drive,
mower, IMU, power, GPS) is simulated, by the `mower_simulation` node.

## Quick start

```bash
# edit .env if you want to test a specific VERSION/APP_VERSION - the checked-in
# defaults work as-is
./sim.sh up
# or, without the wrapper script:
docker compose up -d
```

`mower_simulation_gui` is always built locally (never pulled) on first run - this can
take a few minutes. If you later change source/config that's baked into the image, or
switch `VERSION`/`BASE_IMAGE` in `.env`, re-run with `./sim.sh rebuild` (or
`docker compose up -d --build`) to pick up the change - plain `up` reuses whatever was
already built and won't rebuild on its own.

Then open:
- `http://<host>:6080` - the simulation's RViz view, in your browser (noVNC, no password,
  no VNC client needed). Works from any OS.
- `http://<host>:8080` - the OpenMowerApp web UI.

Comes with a starter mowing area, docking point (`data/ros/map.json`), and mower config
(`data/params/custom_params.yaml`) already checked in, so there's something to drive
around immediately instead of an empty map.

## Notes

- **Startup order**: `mower_simulation_gui` starts first and owns the ROS master;
  `open_mower_ros` waits for it to report healthy (the `mower_simulation` node
  registered) before starting. This matches how a real mower boots - the low-level
  board needs to be there before the ROS logic looks for it.
- **First boot / docking pose**: since the simulator starts before `open_mower_ros`,
  its request for the docking station's position (used to place the robot at start)
  will usually time out and the robot starts at the map origin instead. This is
  cosmetic - reposition it from the app or RViz as needed.
- **Networking**: the whole stack lives on its own Compose-managed bridge network -
  nothing uses host networking, so it won't collide with anything running on your host
  (an existing X display, a VNC server, ports already in use, etc). Only the ports you
  actually need (noVNC/VNC, the app, MQTT) are published to the host.
- **Versions**: `VERSION` controls both the `open_mower_ros` image and the
  `mower_simulation_gui` image (built locally `FROM` that same tag), so the simulator's
  message/service definitions always match the version under test.
- **GPU acceleration**: off by default (works on any host via software rendering). If
  you have a GPU, uncomment the relevant block in `docker-compose.yaml` under
  `mower_simulation_gui` - the container detects it at startup and falls back to
  software rendering automatically if it's not usable.
- **Resetting**: map/params/ROS home persist as plain files under `./data/` (bind
  mounts, not Docker volumes, so `docker compose down -v` doesn't touch them). To go
  back to the checked-in starter map, `./sim.sh reset` (or `git checkout
  data/ros/map.json data/params/custom_params.yaml`); `./sim.sh clean` (or `git clean
  -fdx data/`) wipes everything else (logs, recordings, etc).
- **Native VNC client**: if you'd rather use a VNC client than a browser, connect to
  `<host>:5900` (no password) instead of using noVNC.
- **Wrapper script**: `./sim.sh help` lists shortcuts for the commands on this page
  (`up`, `rebuild`, `logs`, `ps`, `reset`, `clean`, ...) with friendlier error messages
  if Docker isn't installed/running.

## Troubleshooting

```bash
./sim.sh ps                             # check mower_simulation_gui is "healthy"
./sim.sh logs mower_simulation_gui
./sim.sh logs open_mower_ros
```

If the stack was already up and your change doesn't seem to show up, you likely need a
rebuild: `./sim.sh rebuild` (plain `up` never rebuilds on its own).
