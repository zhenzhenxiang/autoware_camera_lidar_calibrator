# Usage

## Camera-LiDAR Extrinsic Calibration

### Calibration

Follow the instructions of the calibration tool from Autoware to get the extrinsic parameters.

### Evalutation

1. Update the camera parameter file in [autoware_camera_lidar_calibrator/data](autoware_camera_lidar_calibrator/data).
2. Launch the script:
   
   ```
   calib_param_yaml:=/path/to/autoware_lidar_camera_calibration.yaml
   ```