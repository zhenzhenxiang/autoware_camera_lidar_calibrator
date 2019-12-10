# Usage

## Camera-LiDAR Extrinsic Calibration

### Calibration

Follow the [instructions](autoware_camera_lidar_calibrator/README.md) of the calibration tool from Autoware to get the extrinsic parameters.

### Evalutation

1. Update the camera parameter file in [autoware_camera_lidar_calibrator/data](autoware_camera_lidar_calibrator/data).
2. Launch the script:
   
   ```
   roslaunch autoware_camera_lidar_calibrator evaluate_camera_lidar_calibration.launch  calib_param_yaml:=/path/to/autoware_lidar_camera_calibration.yaml
   ```