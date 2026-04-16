# beta-0.3.1 Performance Test

## Environment

- Date: 2026-03-29
- Local workspace: `d:\Program\ros2_smart_follower`
- VM: `wheeltec@192.168.220.131`
- Car: `wheeltec@192.168.0.100`
- Car container: `ros2` (`ros2:wheeltec_V1.1`)
- Test rule: restart the car container after each test group to avoid stale ROS processes contaminating data

## Sync And Build Status

### VM

- Workspace synced to `/home/wheeltec/ros2_smart_follower`
- Build command:
  - `source /opt/ros/humble/setup.bash`
  - `source /home/wheeltec/wheeltec_ros2/install/setup.bash`
  - `colcon build --symlink-install --event-handlers console_direct+`
- Result: success
- Summary: `4 packages finished [1min 7s]`

### Car

- Workspace synced to `/home/wheeltec/ros2_shared_dir/ros2_smart_follower`
- Build command:
  - `docker exec ros2 bash -c 'source /opt/ros/humble/setup.bash && source /home/wheeltec/wheeltec_ros2/install/setup.bash && cd /home/wheeltec/ros2_shared_dir/ros2_smart_follower && rm -rf build install log && colcon build --symlink-install --event-handlers console_direct+'`
- Result: success
- Summary: `4 packages finished [1min 40s]`

## Known Issue Confirmed Before Formal Comparison

- `ros2 param set /robot1/perception_node process_every_n_frames 3` will trigger `perception node hot-reloaded, runtime continuing.`
- The perception process then exits with `exit code -11`
- Conclusion: hot-reload path for this parameter is unsafe; comparison tests should use YAML edit + full relaunch, not live param update

## Group 1: Clean Baseline

### Setup

- Car container restarted before launch
- Launch command:
  - `docker exec -d ros2 bash -c 'source /opt/ros/humble/setup.bash && source /home/wheeltec/wheeltec_ros2/install/setup.bash && source /home/wheeltec/ros2_shared_dir/ros2_smart_follower/install/setup.bash && cd /home/wheeltec/ros2_shared_dir/ros2_smart_follower && ros2 launch smart_follower_bringup smart_follower.launch.py robot_ns:=robot1 > /tmp/smart_follower_launch.log 2>&1'`
- Runtime config:
  - `process_every_n_frames: 2`

### Measured Topic Rates

- `/camera/color/image_raw`: `18.017 Hz`
- `/camera/depth/image_raw`: `19.638 Hz`
- `/robot1/person_pose`: `4.576 Hz`

### Diagnostics Snapshot

- `dropped_color_frames`: `684`
- `raw_color_count`: `3967`
- `raw_depth_count`: `4103`
- `queued_color_count`: `3692`
- `skipped_color_frame_count`: `1846`
- `processed_frame_count`: `895`
- `person_pose_publish_count`: `895`
- `position_invalid_ratio`: `0.0189944`
- `profile_last_yolo_ms`: `126.752`
- `profile_last_reid_ms`: `30.5437`
- `profile_last_total_ms`: `163.824`
- `profile_avg_yolo_ms`: `126.908`
- `profile_avg_reid_ms`: `32.619`
- `profile_avg_total_ms`: `165.687`
- `profile_avg_depth_position_ms`: `0.0464681`
- `lock_id`: `-1`
- `lock_state`: `0`

### Host Snapshot

- Temperature: `47.2'C`
- Throttling: `0x0`
- CPU:
  - `perception_node`: `97.5%`
  - `astra_camera_node`: `31.6%`
  - `follower_controller_node`: `1.4%`

### Notes

- This run was collected after a fresh container restart and is valid
- The bottleneck remains perception inference, not depth position projection
- Camera color/depth input is both below nominal 30 FPS on the live system

## Invalid Historical Sample

- A previous `process_every_n_frames=3` run is invalid and must not be used for comparison
- Reason: stale ROS processes survived relaunch and polluted the measurement set

## Group 2: Clean Comparison

### Setup

- Car container restarted before launch
- Runtime config:
  - `process_every_n_frames: 3`
- Launch mode:
  - full stack, same as Group 1

### Measured Topic Rates

- `/camera/color/image_raw`: `17.386 Hz`
- `/camera/depth/image_raw`: `22.832 Hz`
- `/robot1/person_pose`: `4.677 Hz`

### Diagnostics Snapshot

- `dropped_color_frames`: `379`
- `raw_color_count`: `3514`
- `raw_depth_count`: `3611`
- `queued_color_count`: `3369`
- `skipped_color_frame_count`: `2246`
- `processed_frame_count`: `729`
- `person_pose_publish_count`: `729`
- `position_invalid_ratio`: `0`
- `profile_last_yolo_ms`: `194.656`
- `profile_last_reid_ms`: `52.8585`
- `profile_last_total_ms`: `255.201`
- `profile_avg_yolo_ms`: `135.621`
- `profile_avg_reid_ms`: `35.1968`
- `profile_avg_total_ms`: `176.991`
- `profile_avg_depth_position_ms`: `0.0526332`
- `lock_id`: `-1`
- `lock_state`: `0`

### Host Snapshot

- Temperature: `47.7'C`
- Throttling: `0x0`
- CPU:
  - `perception_node`: `94.8%`
  - `astra_camera_node`: `32.1%`
  - `follower_controller_node`: `1.2%`

### Comparison Notes

- This run was also collected after a fresh container restart and is valid
- Compared with Group 1, `person_pose` improved only slightly from `4.576 Hz` to `4.677 Hz`
- `profile_avg_total_ms` got worse from `165.687 ms` to `176.991 ms`
- `profile_avg_yolo_ms` got worse from `126.908 ms` to `135.621 ms`
- `profile_avg_reid_ms` got worse from `32.619 ms` to `35.1968 ms`
- `process_every_n_frames=3` is not a worthwhile optimization under full-stack load on the current hardware

## Group 3: Camera + Perception Only

### Setup

- Car container restarted before launch
- Runtime config:
  - `process_every_n_frames: 2`
- Launch mode:
  - only `astra_camera`
  - only `perception_node`
- Not launched:
  - `wheeltec_robot_node`
  - `ekf_node`
  - `follower_controller_node`
  - `ultrasonic_range_node`
  - `obstacle_avoidance_node`
  - `arbiter_node`
  - `keyboard_command_node`
  - `robot_state_publisher`
  - `joint_state_publisher`

### Measured Topic Rates

- `/camera/color/image_raw`: `11.016 Hz`
- `/camera/depth/image_raw`: `23.639 Hz`
- `/robot1/person_pose`: `5.153 Hz`

### Diagnostics Snapshot

- `dropped_color_frames`: `178`
- `raw_color_count`: `2918`
- `raw_depth_count`: `2969`
- `queued_color_count`: `2854`
- `skipped_color_frame_count`: `1427`
- `processed_frame_count`: `617`
- `person_pose_publish_count`: `617`
- `position_invalid_ratio`: `0`
- `profile_last_yolo_ms`: `129.018`
- `profile_last_reid_ms`: `34.2186`
- `profile_last_total_ms`: `168.531`
- `profile_avg_yolo_ms`: `132.387`
- `profile_avg_reid_ms`: `34.1854`
- `profile_avg_total_ms`: `172.62`
- `profile_avg_depth_position_ms`: `0.0446105`
- `lock_id`: `-1`
- `lock_state`: `0`

### Host Snapshot

- Temperature: `47.7'C`
- Throttling: `0x0`
- CPU:
  - `perception_node`: `99.0%`
  - `astra_camera_node`: `31.4%`

### Comparison Notes

- Removing the rest of the robot stack did not release a meaningful amount of CPU for perception
- `person_pose` improved from `4.576 Hz` to `5.153 Hz`, but the gain is small
- `perception_node` remained saturated near `100%` CPU
- Depth throughput improved clearly, but color throughput became worse in this sample
- Current evidence still points to the perception pipeline itself as the dominant bottleneck, with possible color-camera side scheduling or transport instability

## Preliminary Conclusion

- On Raspberry Pi 5B, the current `640x480 + color/depth + YOLO + ReID + full stack` performance is broadly consistent with a CPU-bound pipeline
- The dominant cost is still `YOLO + ReID`
- `process_every_n_frames=3` does not provide a useful win in the current architecture
- The next useful optimization direction is not further tuning follower/control nodes, but reducing perception-side compute or color pipeline pressure

## Group 4: Full Stack With `process_every_n_frames=1`, `detect_every_n_frames=2`

### Setup

- Car container restarted before launch
- Runtime config:
  - `process_every_n_frames: 1`
  - `detect_every_n_frames: 2`
- Code state:
  - before queue fix
  - original single `pending_work_` slot
  - original single `latest_result_` slot

### Measured Topic Rates

- `/camera/color/image_raw`: `19.600 Hz`
- `/camera/depth/image_raw`: `23.651 Hz`
- `/robot1/person_pose`: `5.018 Hz`

### Diagnostics Snapshot

- `dropped_color_frames`: `461`
- `queued_color_count`: `2565`
- `processed_frame_count`: `624`
- `person_pose_publish_count`: `624`
- `profile_detect_frames`: `293`
- `profile_avg_detect_interval_frames`: `2.12969`
- `profile_avg_yolo_ms`: `61.1526`
- `profile_avg_reid_ms`: `13.5528`
- `profile_avg_total_ms`: `80.297`
- `profile_last_total_ms`: `8.90713`

### Host Snapshot

- Temperature: `47.7'C`
- Throttling: `0x0`
- CPU:
  - `perception_node`: `83.5%`
  - `astra_camera_node`: `30.9%`

### Interpretation

- Lowering detection frequency was directionally correct
- But `person_pose` still only reached about `5 Hz`
- This exposed that the pipeline was dropping work internally, not just running too slowly

## Group 5: After Pending-Work Queue Fix

### Code Change

- Replaced single pending work slot with a bounded pending work queue in `PerceptionPipeline`
- Added diagnostic counter:
  - `dropped_pending_work_count`

### Measured Topic Rates

- `/camera/color/image_raw`: `19.535 Hz`
- `/camera/depth/image_raw`: `18.975 Hz`
- `/robot1/person_pose`: `6.100 Hz`

### Diagnostics Snapshot

- `dropped_color_frames`: `334`
- `queued_color_count`: `2050`
- `dropped_pending_work_count`: `1245`
- `processed_frame_count`: `525`
- `person_pose_publish_count`: `525`
- `profile_detect_frames`: `228`
- `profile_avg_detect_interval_frames`: `2.30263`
- `profile_avg_yolo_ms`: `57.2127`
- `profile_avg_reid_ms`: `14.8829`
- `profile_avg_total_ms`: `78.0972`

### Host Snapshot

- Temperature: `46.1'C`
- Throttling: `0x0`
- CPU:
  - `perception_node`: `100%`
  - `astra_camera_node`: `32.1%`

### Interpretation

- The pending-work queue improved effective publish rate from about `5.0 Hz` to about `6.1 Hz`
- The new counter confirmed that the worker was still dropping many synchronized frames before processing
- This indicated a second bottleneck on the result handoff path

## Group 6: After Pending-Work Queue + Ready-Result Queue Fix

### Code Change

- Kept the bounded pending work queue
- Replaced single ready result slot with a bounded ready result queue
- Changed the result timer callback to drain all currently ready results
- Added diagnostic counter:
  - `dropped_ready_result_count`

### Measured Topic Rates

- `/camera/color/image_raw`: `22.708 Hz`
- `/camera/depth/image_raw`: `21.442 Hz`
- `/robot1/person_pose`: `9.015 Hz`

### Diagnostics Snapshot

- `dropped_color_frames`: `311`
- `queued_color_count`: `1949`
- `dropped_pending_work_count`: `1191`
- `dropped_ready_result_count`: `0`
- `processed_frame_count`: `755`
- `person_pose_publish_count`: `755`
- `profile_detect_frames`: `468`
- `profile_avg_detect_interval_frames`: `1.61325`
- `profile_avg_yolo_ms`: `84.2559`
- `profile_avg_reid_ms`: `22.1082`
- `profile_avg_total_ms`: `112.869`
- `profile_last_total_ms`: `186.847`

### Host Snapshot

- Temperature: `47.7'C`
- Throttling: `0x0`
- CPU:
  - `perception_node`: `99.1%`
  - `astra_camera_node`: `32.2%`

### Interpretation

- `person_pose` improved again from about `6.1 Hz` to about `9.0 Hz`
- `dropped_ready_result_count=0` shows the result queue successfully eliminated the second overwrite point
- The dominant remaining loss is now still on the input side:
  - synchronized frames are still being dropped before worker processing
- This is the first tested configuration that clearly exceeds the original goal floor of roughly `7 Hz` inference-equivalent output under full-stack load

## Updated Conclusion

- The most effective optimization so far is:
  - `process_every_n_frames=1`
  - `detect_every_n_frames=2`
  - bounded pending work queue
  - bounded ready result queue with timer draining
- On the current full stack, this moved `person_pose` from roughly `4.6 Hz` baseline to roughly `9.0 Hz`
- Camera topic rates also improved into the `21~23 Hz` range in the latest run, which is very close to the practical `21 Hz` perception target
- The remaining headroom is still limited by perception CPU saturation, especially YOLO/ReID on detect frames

## 2026-04-02 Lower-Body Depth Sampling Patch

### Goal

- Reduce close-range `person_pose` NaN frames with a lightweight depth-sampling strategy
- Keep CPU overhead very low on Raspberry Pi 5B

### Code Change

- Changed depth sampling in `smart_follower_perception/src/geometry_utils.cpp`
- Replaced the old single-column vertical sampling with sparse lower-body anchors:
  - `x ratios = {0.35, 0.50, 0.65}`
  - `y ratios = {0.62, 0.74, 0.86}`
- Sampling now focuses on the lower half of the person box instead of the torso/center only
- Added per-pixel dedup across overlapping sample windows so valid-sample counts are not artificially inflated
- This keeps the algorithm simple: 9 sparse windows, median depth, no heavy fitting or temporal computation

### Test Update

- Updated geometry unit tests to match the new lower-body sampling anchors
- Added a side-window recovery case where center samples are empty but lower-body side samples still recover valid depth

### Car Build Verification

- Synced package: `smart_follower_perception`
- Build target: car container `ros2`
- Build command:
  - `docker exec ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /home/wheeltec/wheeltec_ros2/install/setup.bash && cd /home/wheeltec/ros2_shared_dir/ros2_smart_follower && colcon build --symlink-install --packages-select smart_follower_perception --event-handlers console_direct+'`
- Result: success

### Car Test Verification

- Test command:
  - `docker exec ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /home/wheeltec/wheeltec_ros2/install/setup.bash && cd /home/wheeltec/ros2_shared_dir/ros2_smart_follower && colcon test --packages-select smart_follower_perception --event-handlers console_direct+ && colcon test-result --verbose --test-result-base build/smart_follower_perception'`
- Result: success
- Summary: `23 tests, 0 errors, 0 failures, 0 skipped`

### Next Observation Focus

- Re-test close-range lock scenarios on the car
- Compare whether these warnings become less frequent:
  - `depth_window_no_valid_samples`
  - `locked_track_position_nan`
- Especially observe people near the image top edge or with large/clipped boxes

## 2026-04-03 VM Validation: Low-Mounted Leg-Focused Depth Sampling

### Goal

- Optimize depth sampling for low camera view that mainly sees legs
- Keep algorithm lightweight and robust for normal front-follow posture

### Algorithm Update

- Sampling anchors changed to leg-focused sparse points inside lower body
- X anchors: `0.32`, `0.68` (two leg columns)
- Y anchors: `0.64`, `0.74`, `0.84`, `0.90` (lower body only)
- Kept small window sampling and per-pixel dedup across overlapping windows
- Depth aggregation changed from median to near-depth quantile `P35`
  - better resistance to background leakage through leg gaps

### Code Paths

- `src/smart_follower_perception/src/geometry_utils.cpp`
- `src/smart_follower_perception/test/test_geometry_utils.cpp`

### VM Build/Test

- VM workspace: `/home/wheeltec/ros2_smart_follower`
- Build command:
  - `colcon build --symlink-install --packages-select smart_follower_perception --event-handlers console_direct+`
- Result: success
- Test command:
  - `colcon test --packages-select smart_follower_perception --event-handlers console_direct+`
  - `colcon test-result --verbose --test-result-base build/smart_follower_perception`
- Result: success
- Summary: `23 tests, 0 errors, 0 failures, 0 skipped`
