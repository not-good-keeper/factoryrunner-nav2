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
- **Still open:** one Fast DDS run (162513) also failed leg 4 near the keepout lane with MPPI "Optimizer fail to compute path" before the container froze. It has not recurred under Cyclone. The keepout filter comes after the inflation layer in both costmaps, so the lane is not inflated; whether that caused the failure is UNVERIFIED. The mission controller sometimes stays alive after printing MISSION COMPLETE; that moves to the per-leg timeout task.

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
