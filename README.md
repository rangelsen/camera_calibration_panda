### Camera Calibration Panda
## How to calibrate the cameras
- Open a terminal window and navigate to project build directory

- Build the source by running `make`

- Capture the calibration images of the ChArUco board by running `./calib_manual <name-of-dataset>` and following the instructions as prompted

- Navigate to `res/<name-of-dataset>` and copy the calibration script by running the command `cp ../../src/estimate_camera_pose.m .`.

- Run the calibration by executing `octave estimate_camera_pose.m <camera-serial-number>` for each camera in the rig. The camera pose in the robot base frame has now been written to the file named bTc_<camera-serial-number>.csv

- In order to verify that the calibration is correct you can display the camera poses in the base frame as well as the robot base. In addition you can plot the depth images as point clouds in the base frame in a 3D virtual environment using the respective camera poses. You can now see if the two images of the same scene overlap or not. This step is optional but highly recommended.

## Execute order
- calib_manual
- estimate_camera_pose.m
- verification
- pointcloud-view
