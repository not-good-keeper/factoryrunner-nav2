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
