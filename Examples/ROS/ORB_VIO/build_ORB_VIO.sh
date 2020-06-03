echo ""
echo "Building ROS ORB_VIO nodes"

echo "...."
echo "Step1: Building ORB_SLAM nodes"
cd ~/rosbuild_ws/package_dir/ORB_SLAM2_OpenCV3.2/build/
# cmake .. -DROS_BUILD_TYPE=Release
make -j4

echo ""
echo "Step2: Building ORB_VIO nodes"
cd ~/rosbuild_ws/package_dir/ORB_SLAM2_OpenCV3.2/Examples/ROS/ORB_VIO/build/
# cmake .. -DROS_BUILD_TYPE=Release
make -j4
