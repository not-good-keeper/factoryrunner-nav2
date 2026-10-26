# Challenges log

One entry per real problem: date, symptom, what I checked, what fixed it.

## 2026-10-26 — Existing WSL distro was Ubuntu 20.04

- **Symptom:** the WSL home I had been using is Ubuntu 20.04. ROS 2 Jazzy only ships binaries for Ubuntu 24.04, and Humble (22.04) uses Gazebo Classic for the TB3 sim.
- **Plan:** install a separate Ubuntu-24.04 distro alongside it rather than upgrading the old one in place. Update this entry once it works.

## 2026-10-26 — Keepout and speed zones silently did nothing

- **Symptom:** after adding the keepout and speed filters, the costmaps logged `KeepoutFilter: Filter mask was not received` every few seconds and the planned path still went straight through the forklift lane.
- **Checked:** `ros2 topic info /keepout_filter_mask -v` showed the mask server publishing (reliable, transient local). The filter info server was publishing on `/keepout_filter_info`. The costmap log said it was subscribing to `"keepout_filter_info"`.
- **Cause:** the topic names in `nav2_params.yaml` were relative. The costmaps run as `/global_costmap/global_costmap` and `/local_costmap/local_costmap`, so they were listening on `/global_costmap/keepout_filter_info`, which nothing publishes. The same applied to the speed filter's `speed_limit` output, which the controller reads from `/speed_limit`.
- **Fix:** made every filter topic absolute (`/keepout_filter_info`, `/speed_filter_info`, `/speed_limit`, and the mask topics in the launch file). Both costmaps then logged `Received filter mask`; the next planned path crossed x = 2.4 m at y ≈ −2.8 m, below the lane, and `/speed_limit` published 40 % when the robot entered the CNC zone.

## 2026-10-26 — Missions froze or aborted partway through (two separate faults)

- **Symptom:** on the RTX 3060 machine, leg 1 succeeded, then the robot sat still on leg 2 with 9.61 m to go, and `tf2_echo map odom` reported that `map` did not exist. The cloud VM had failed on the same leg with `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`. I suspected AMCL.
- **Reproduced headless** with the committed config (Fast DDS 2.14.7, `default_server_timeout: 20`). Gazebo's real-time factor was 1.0, so the CPU was keeping up. None of 3 runs completed: 2 ended with the Nav2 container frozen, 1 aborted on leg 1.
- **Fault 1, the container froze.** In the stuck state `/scan` still ran at 4.9 Hz, but the `map` frame was gone, `ros2 lifecycle get /amcl` returned "Node not found", and every Nav2 node had disappeared from `ros2 node list`. Yet `component_container_isolated` was still alive, at 0 % CPU in one run. AMCL had not crashed. `ptrace_scope=1` blocks attaching a debugger, so I started the container under gdb through `ros2 launch --launch-prefix` and had SIGUSR1 dump every thread at the next freeze. Of the 75 threads, 20 were blocked on Fast DDS locks: 12 waiting for a send buffer (`SendBuffersManager::get_buffer`), 7 on a mutex in `StatefulReader::processDataMsg` called from `StatefulWriter::intraprocess_delivery`, and 1 in discovery cleanup (`StatefulWriter::matched_reader_remove` from `PDP::actions_on_remote_participant_removed`). About 49 executor threads sat idle in DDS wait-sets, and the rest were helpers such as the signal handler and MPPI's noise thread. No Nav2 code was on a blocked path. With the middleware stuck, AMCL could not publish map→odom and the navigator got no replies, which fits what both machines showed.
- **Fix 1:** Cyclone DDS (`ros-jazzy-rmw-cyclonedds-cpp`, `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) in the setup script and README; `mission_controller` warns when the variable is not set. Count so far: froze in 2 of 3 runs under Fast DDS 2.14.7, in 0 of 9 under Cyclone. That is evidence, not proof of a Fast DDS bug: I did not find which lock ordering deadlocks, and the samples are small.
- **Fault 2, one late acknowledgement aborted the goal.** Seen in 1 of 3 Fast DDS runs and 1 of 3 Cyclone runs at 20 ms, so it does not depend on the middleware. The navigator gives the planner 20 ms to acknowledge each replanning request. If the reply is later, `BtActionNode` returns FAILURE without setting `compute_path_error_code`. Both recovery branches in Jazzy's `navigate_to_pose_w_replanning_and_recovery.xml` start with `WouldAPlannerRecoveryHelp` / `WouldAControllerRecoveryHelp`, which pass only for specific error codes, so the tree aborted 30 ms after the warning with no costmap clear, spin or wait.
- **Fix 2:** `default_server_timeout: 200` in `bt_navigator`.
- **Result:** with both fixes, 3 consecutive headless missions completed 4/4 (sim time 95.3, 92.0 and 90.7 s), with one Gazebo server running and no acknowledgement timeouts in the logs.
- **Test-harness problem found on the way:** one `gz sim` server ignored SIGTERM and stayed alive under the next five launches. Those runs are marked below and are not counted as the result. The stop step now escalates to SIGKILL and refuses to continue while any Gazebo or Nav2 process is left.
- **Still open at the time:** one Fast DDS run (162513) also failed leg 4 near the keepout lane with MPPI "Optimizer fail to compute path" before the container froze. It recurred under Cyclone; see the next entry. The mission controller sometimes stays alive after printing MISSION COMPLETE; that moves to the per-leg timeout task.

Runs in `results/missions.csv` (all headless):

| run_id | Middleware | Timeout | Outcome |
|---|---|---|---|
| 20261026-161716 | Fast DDS | 20 ms | leg 1 aborted on an acknowledgement timeout |
| (not in CSV) | Fast DDS | 20 ms | legs 1–3 succeeded, container froze on leg 4; controller killed before it wrote the CSV |
| 20261026-162513 | Fast DDS | 20 ms | container under gdb; leg 4 failed (MPPI), container froze afterwards |
| 20261026-163132 | Cyclone | 20 ms | 4/4 |
| 20261026-163610 | Cyclone | 20 ms | 4/4, stale Gazebo server alive |
| 20261026-164035 | Cyclone | 20 ms | leg 1 aborted on an acknowledgement timeout, stale Gazebo server alive |
| 20261026-164746, -165226, -165651 | Cyclone | 200 ms | 4/4 each, stale Gazebo server alive |
| 20261026-170336, -170600, -170819 | Cyclone | 200 ms | 4/4 each, one Gazebo server |

## 2026-10-26 — Robot stuck on the keepout lane's corner

- **Symptom:** on leg 4 (INSPECTION → DOCK), two runs stopped with exactly 4.01 m of path left: one under Fast DDS (162513) and one under Cyclone with both fixes above (171949). The controller logged `Optimizer fail to compute path` until `Controller patience exceeded`, and the spin recovery quit with `Collision Ahead`.
- **Checked:** with the robot still stuck in the Cyclone run, `tf2_echo map base_link` gave (2.778, 2.503). The keepout lane covers x 2.0–2.8, y −2.5 to 2.5, so the robot's centre was 3 mm from the lane's corner and its 0.22 m radius overlapped lethal cells. MPPI treats every trajectory from there as a collision.
- **Cause:** `keepout_filter` came after `inflation_layer` in both costmaps' plugin lists. The inflation layer only inflates what is lethal in the master grid when it runs, so walls got a 0.7 m inflation band and the lane got none. NavFn saw zero cost right up to the lane edge and planned along it; MPPI then cut the corner.
- **Fix:** moved `keepout_filter` before `inflation_layer` in both costmaps. A probe of the live global costmap afterwards read (OccupancyGrid scale) 100 inside the lane; 99 at 0.1 m above its top edge and 0.2 m beside its side edge; 78 at 0.3 m above or below it; 58 at 0.4 m beside it; 43 at 0.5 m above it. I did not probe the old order, so there is no measured "before" value.
- **Result:** 5 of 5 headless missions completed 4/4 (sim time 91.2–92.2 s) with no MPPI failures and no recoveries (runs 172350, 172606, 172815, 173029, 173237). Before the change it failed in 2 of the 11 runs that got as far as that corner on leg 4 (counting one GUI run on the user's machine), so 0 of 5 is consistent with a fix but is not yet strong evidence; the trial runs will add more.

## 2026-10-27 — Gazebo's create and remove replies don't confirm anything

- **Symptom:** while testing `factoryrunner/obstacle.py`, spawning a second box with the same name and removing a box that was already gone both reported success. `ros_gz_sim create` printed `Entity creation successful.` and exited 0; `/world/depot/remove` replied `data: true`.
- **Checked:** the Gazebo server log showed `Entity named [pallet_box] of type [2] not found, so not removed.` for the second removal, so the server knew; the service reply was sent when the request was queued, before the world processed it. I then looked for a way to confirm the entity list: `gz model --list` timed out on `/world/depot/state`, and the headless server publishes no `scene/info` or `pose/info`, because it runs without a scene broadcaster.
- **Decision:** the helper reports "requested" rather than "spawned", raises only when the command itself fails, and the check moves to the robot's side. In a leg-2 test with the box 3 m ahead on the plan, the global costmap read 99 (inscribed) at the box centre, which also confirms the map → Gazebo offset of (−8, 0); every plan after the first two kept about 0.9–1.0 m from the box, and the leg took 29.9 s against about 29.3 s without it. With the box 1.2 m ahead, the collision monitor reported SlowdownZone and briefly FootprintApproach, the robot passed with about 9 cm of air gap, and the leg took 54.4 s.
