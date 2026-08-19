# ros1_bridge (NeoFarm fork)

Base: <https://github.com/ros2/ros1_bridge> `master`, commit
`611755fd917285316051cbea80507e8b2f6b7ec1`
("Fix REP url locations", #463, 2025-11-17). Upstream
`master` has not moved since; the newest release tag is still `0.9.7`
(2023) and no branch tracks Kilted, so `master` IS the rolling branch.

On top of that base, in order:

1. Three Kilted build fixes from
   <https://github.com/mikaelarguedas/ros1_bridge> branch
   `fix_some_warnings` (upstream PRs #411, #434, both still open):
   `ce86889`, `4f36cb1`, `5cd15e5`.
2. The NeoFarm island-topology changes (list below).
3. One cherry-pick from an open upstream PR (#468).

Rebased onto `611755f` on 2026-08-19 (previous base: `3d5328d`,
2022). The rebase changed **no code** — see "Rebase history" below.

Forked locally 2026-08-17 so the island-topology fixes live as
reviewable commits instead of build-time patching (previously
`docker/bridge_patches/patch_ros1_bridge.py` in neofarm_ros). This repo
is a WORKSPACE SIBLING of neofarm_ros (`<catkin_ws>/src/ros1_bridge`),
built exclusively by neofarm_ros `docker/ros1_bridge.Dockerfile` via an
extra BuildKit context (`--build-context ros1_bridge_src=../ros1_bridge`
from the neofarm_ros repo root). Keep an untracked `CATKIN_IGNORE` file
at this repo's root so the ROS 1 workspace colcon build never crawls it
(the Dockerfile strips the marker on copy).

Platform: Ubuntu 24.04 + ROS-O (`/opt/ros/one`) + ROS 2 Kilted. The
upstream README compatibility matrix marks 24.04 "not supported" — that
is a statement about official ROS 1 packaging, not about buildability.
Open upstream PR #462 adds a CI matrix that builds and tests this very
combination (ROS-O + Jazzy on noble) green.

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
5. **Executor survives forward exceptions** (`src/parameter_bridge.cpp`).
   A failed LOCAL ROS 1 service call inside `forward_2_to_1` throws; the
   exception escapes `executor.spin()` and terminates the whole bridge
   process (observed live: greenhouse bridge died on
   `Failed to get response from ROS 1 service /api/action/get`). The spin
   is wrapped in a catch-and-continue loop — the unanswered ROS 2 request
   times out on the caller side (the `b''` semantics callers already
   handle) and every other relay keeps running.
6. **Loopback filter uses the bridge's real node name**
   (`include/ros1_bridge/factory.hpp`), cherry-picked from open upstream
   PR #468. The 1→2 relay's own-traffic filter compared the ROS 1
   connection-header callerid against the hardcoded `"/ros_bridge"`,
   which is only the `ros::init` default — any `__name:=` override or a
   second bridge on one master defeats the filter and doubles the ROS 2
   rate. A no-op today (every island runs one unnamed bridge), taken
   pre-emptively because the failure mode is silent duplication.

## Upstream PRs evaluated 2026-08-19

`master` is dormant but the PR queue is not, and several open PRs solve
the same problems as changes 1–5. Re-check this table before adopting
anything: taking a PR that duplicates a local change means resolving a
conflict for no behaviour gain.

| PR | Subject | Verdict |
| --- | --- | --- |
| #468 | loopback filter node name | **taken** — change 6 |
| #411, #434 | `vector<bool>`, deprecated CMake/typesupport | already in, via `mikaelarguedas` |
| #442 | latch ROS 1 pubs | superseded by change 1 (blanket latch vs. per-topic durability) |
| #438, #424 | `tf_static` special case | superseded by changes 1+2, which generalise it via allowlist QoS. Neither PR fixes the real remaining gap (below) |
| #315 | multi-threaded services | superseded by change 3. Also `CONFLICTING` upstream and it rewrites `factory.hpp`, which change 1 owns |
| #446, #462 | `vector<bool>` via C++ overloads / SFINAE | redundant. Cleaner than the `streamPrimitiveVectorBool` + `typename == 'boolean'` template branch inherited from #411, but functionally the same fix for the same compile error, so swapping buys nothing. #462 WILL conflict here if it merges |
| #467 | manual field-mapping rules ignored | not applicable — NeoFarm defines no `*_mapping_rules.yaml` |
| #397 | log missing pairs once | no effect — touches `dynamic_bridge` only; we run `parameter_bridge` |
| #462 (CI part) | ROS-O CI matrix | do not take: pins `iory/*` action forks, and its `set(ament_cmake_pep257_FOUND TRUE)` disables a lint globally to work around a focal/py3.8 issue we do not have |

### Note on the `vector<bool>` commits

`ce86889` + `4f36cb1` exist because upstream cannot compile a message
with a `bool[]` field (`std::vector<bool>` has no addressable elements,
so the `memcpy` in `streamPrimitiveVector` does not compile). They came
in with the `mikaelarguedas` branch, not from a NeoFarm requirement: no
`nfar_*` msg/srv declares a `bool[]`. The path is only instantiated when
a BRIDGED pair carries one, so on a vanilla base-messages build it is
compiled-but-never-called. Do not assume it is exercised by the current
production image without checking the generated factories for
`streamPrimitiveVectorBool(stream,` call sites.

### Known gap: `/tf_static` accumulation

ROS 1 latching replays only the LAST message per publisher. Every ROS 2
static broadcaster relayed 2→1 goes out through the bridge's single ROS 1
`/tf_static` publisher, so a late-joining ROS 1 subscriber sees only
whichever broadcaster published most recently. The ROS 2 side is fine
(DDS transient_local history is per-writer, so `depth: 1` still delivers
the latest sample from each broadcaster). Fixing this needs the bridge to
accumulate `TFMessage.transforms` across relayed messages and republish
the union — no upstream PR does this. Not currently biting because the
allowlist frames are published before consumers attach.

## Updating

1. `git fetch upstream && git rebase upstream/master`.
2. Expect a conflict in `resource/interface_factories.cpp.em`: upstream
   #456 fixes the service-field argument order that `4f36cb1` already
   fixed by deleting the alias variables #456 patches. **Keep ours** —
   verify with `git diff <old-tip> HEAD -- . ':!README.md' ':!doc'
   ':!.github'`, which should come back empty.
3. Re-check the PR table above.
4. Update the base commit and date at the top of this file.

Keep the diff reviewable: no unrelated reformatting of upstream files.

## Rebase history

| Date | From | To | Code delta |
| --- | --- | --- | --- |
| 2026-08-19 | `3d5328d` (#392, 2022) | `611755f` (#463, 2025-11-17) | none — README, `doc/index.rst`, deleted issue template only |
