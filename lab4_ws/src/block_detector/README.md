
# RGB-D Block Detector

```bash
sudo apt update

sudo apt install \
    python3-opencv \
    python3-numpy \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-rqt-image-view \
    ros-humble-rosbag2
```

```bash
cd ~/ros2_cv_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select block_detector

source install/setup.bash
```

Open three WSL terminals.

### Terminal 1: Play the Bag

```bash
source /opt/ros/humble/setup.bash

ros2 bag play \
    ~/megn441dev/block_detector/test_data/rgbd_test_5sec \
    --loop
```

### Terminal 2: Run the Detector

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_cv_ws/install/setup.bash

ros2 run block_detector block_detector
```

### Terminal 3: View the Results

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_cv_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view
```

Select this topic in the image viewer:

```text
/block_detector/debug_image
```

Stop any running test image publisher before playing the bag.

Press Ctrl+C to stop playback or the detector.

## Run the Live Camera on the JetRover

```zsh
source /opt/ros/humble/setup.zsh

source ~/third_party/orbbec_ws/install/local_setup.zsh
```

```zsh
ros2 launch orbbec_camera dabai_dcw.launch.py \
    depth_registration:=true \
    align_mode:=HW
```

### Run the Detector on the Robot

```zsh
source /opt/ros/humble/setup.zsh

source ~/ros2_cv_ws/install/local_setup.zsh

ros2 run block_detector block_detector
```