# ros_dual2equi

This package converts dual-fisheye images into equirectangular format using precomputed pixel coordinate mappings. The provided mappings are optimized for 1280x960 resolution, based on the Ricoh Theta S camera calibration from the MIS lab.

## Installation

1. Clone the repository into your ROS catkin workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/GraceSevillano/ros_dualfisheye2equi.git
   ```

2. Build and source the package:

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage

To run the package:

```bash
roslaunch ros_dual2equi dual2equi_bgr8.launch bagfile:=/path/to/your.bag output_image_path:=/path/to/output
```

### Parameters

- **bagfile:** Path to the input ROS bag file.
- **inputImagesTopic:** Topic for dual-fisheye images.
- **outputImagesTopic:** Topic for the equirectangular output.
- **coordinatesTableFilename:** Path to the mapping table.
- **imWidth / imHeight:** Image resolution settings.
- **queue_size:** Queue size for image processing.
- **use_sim_time:** Enable simulated time.

### Outputs

- Equirectangular images saved to the specified output path.
- `rgb.txt` file containing timestamps and corresponding image filenames.

## Acknowledgment

This fork is based on the original work by **G. Caron**, with additional improvements to enhance usability.

For further mapping computations, see [libPeR](https://github.com/PerceptionRobotique/libPeR_base).



