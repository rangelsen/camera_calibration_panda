### Camera Calibration Panda
## How to calibrate the cameras
- Open a terminal window and navigate to project build directory

- Build the source by running `make`

- Capture the calibration images of the ChArUco board by running `./calib_manual <name-of-dataset>` and following the instructions as prompted

- Navigate to `res/<name-of-dataset>` and copy the calibration script by running the command `cp ../../src/estimate_camera_pose.m .`.

- Run the calibration by executing `octave estimate_camera_pose.m <camera-serial-number>` for each camera in the rig

## Execute order
- calib_manual
- estimate_camera_pose.m
- verification
- pointcloud-view
