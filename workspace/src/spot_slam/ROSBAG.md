## Using Rosbag

The following are some examples of running the spot_slam package over standardised datasets.
You can use rosbag if you don't have access to an iOS device.

### Mono mode with [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)'s [`eee_01.bag`](https://researchdata.ntu.edu.sg/api/access/datafile/68133)

```bash
# Terminal 1:
roslaunch orb_slam3_ros ntuviral_mono.launch
# Terminal 2:
rosbag play eee_01.bag -s 50 # The UAV starts moving at t~50s
```

### Mono-inertial mode with [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)'s [`MH_01_easy.bag`]( http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag)

```bash
# Terminal 1:
roslaunch orb_slam3_ros euroc_mono_inertial.launch
# Terminal 2:
rosbag play MH_01_easy.bag
```

### RGB-D mode with [TUM](http://vision.in.tum.de/data/datasets/rgbd-dataset/download)'s [`rgbd_dataset_freiburg1_xyz.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag)

```bash
# Terminal 1:
roslaunch orb_slam3_ros tum_rgbd.launch
# Terminal 2:
rosbag play rgbd_dataset_freiburg1_xyz.bag
```

- **Note**: change `TUMX.yaml` to `TUM1.yaml`,`TUM2.yaml` or `TUM3.yaml` for freiburg1, freiburg2 and freiburg3 sequences respectively.

### RGB-D-Inertial mode with [VINS-RGBD](https://github.com/STAR-Center/VINS-RGBD)'s [`Normal.bag`](https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag)

- Download the bag files, for example [Normal.bag](https://star-center.shanghaitech.edu.cn/seafile/d/0ea45d1878914077ade5/).
- Decompress the bag, run `rosbag decompress Normal.bag`.
- Change the calibration params in `RealSense_D435i.yaml` if necessary.

```bash
# Terminal 1:
roslaunch orb_slam3_ros rs_d435i_rgbd_inertial.launch.launch
# Terminal 2:
rosbag play Normal.bag
```
