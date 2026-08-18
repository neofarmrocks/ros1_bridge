# ros1_bridge (NeoFarm fork)

Upstream: <https://github.com/mikaelarguedas/ros1_bridge>, branch
`fix_some_warnings`, commit `75e91bd8ef5c2a296bd65364f586ca43d856cabe`
(itself a fork of <https://github.com/ros2/ros1_bridge> carrying three
Kilted build fixes: `39c87999`, `77bdc4c2`, `75e91bd8`).

Forked locally 2026-08-17 so the island-topology fixes live as
reviewable commits instead of build-time patching (previously
`docker/bridge_patches/patch_ros1_bridge.py` in neofarm_ros). This repo
is a WORKSPACE SIBLING of neofarm_ros (`<catkin_ws>/src/ros1_bridge`),
built exclusively by neofarm_ros `docker/ros1_bridge.Dockerfile` via an
extra BuildKit context (`--build-context ros1_bridge_src=../ros1_bridge`
from the neofarm_ros repo root). Keep an untracked `CATKIN_IGNORE` file
at this repo's root so the ROS 1 workspace colcon build never crawls it
(the Dockerfile strips the marker on copy).

## Local changes vs upstream

Each is load-bearing for the one-rosmaster-per-machine topology
(distributed simulator now, per-machine farm bridges later):

1. **Per-topic latch on the bridge's ROS 1 republishers**
   (`include/ros1_bridge/factory.hpp`, `factory_interface.hpp`,
   `src/bridge.cpp`). With a machine-local master the bridge is the ONLY
   publisher of a relayed latched topic; `latch=false` loses
   publish-on-change state (`locked_garden`, `/tf_static`, chapel frames)
   for every late joiner. Latching everything is equally wrong (stale
   event topics become poison pills — e.g. the spooler abort Empty), so
   the latch mirrors the configured per-topic durability
   (`transient_local` ⇒ latch). The default flips in BOTH headers because
   C++ resolves virtual default arguments from the static type.
2. **2→1 ROS 2 subscriber inherits the configured per-topic QoS**
   (`src/bridge.cpp`). Upstream hardcodes SensorDataQoS
   (volatile + best-effort), so a (re)started consuming bridge never
   backfills transient_local history. Also feeds change 1.
3. **Multi-threaded executor** (`src/parameter_bridge.cpp`). Service
   forwards block their executor thread for up to
   `service_execution_timeout`; a caller retry loop (the starter's 4 s
   resend) starves the single thread, freezing every topic relay the
   bridge carries — `/clock` included, which stalls sim time on all
   consuming machines.
4. **Directional topic entries** (`src/parameter_bridge.cpp`). Upstream
   only builds BIDIRECTIONAL topic bridges, so every island bridge also
   injects a ROS 1 echo of each relayed topic back into the ROS 2 graph.
   For one-authority topics (`/clock`: the rclcpp simulated clock) that
   means N bridge publishers competing with the authority — observed to
   stall the 2→1 clock relay on consuming islands. An optional per-topic
   `direction` key (`2_to_1` or `1_to_2`, default bidirectional) creates
   a single-direction bridge instead. `service_execution_timeout` is
   read from `ros1_bridge/parameter_bridge/service_execution_timeout`
   (set by `docker/run_parameter_bridge.sh`).

## Updating

Pull the new upstream, re-apply the four changes (they are small and
documented above), and update the commit hash here. Keep the diff
reviewable: no unrelated reformatting of upstream files.
