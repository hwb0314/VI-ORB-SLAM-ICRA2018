#!/bin/bash
# created by Huangweibo
# Date: 2017年7月22日
# Description：自动执行Vi-ORB_SLAM2程序，执行5次
# Input:
# 	dataname	# 数据序列名字
# 	datadir		# 数据序列所在文件夹
# 	yaml		# 数据对应的相机参数
#	downSampleRate	# 下采样设置，默认1
#	keyframeTrajectorSavePath	# 轨迹保存路径
# Example：
#	dataname="MH_01_easy"
#	datadir="Downloads/SLAM_Dataset/Dataset_from_TUM/Handheld_SLAM/$dataname"
#	bagfile: "/home/hri/Documents/SLAM_Dataset/EuRoC/rosbag_file/MH_01_easy.bag"
#	yaml="./Config/EuRoC.yaml"
#	keyframeTrajectorSavePath="keyframeTrajectorAndATE_Original"
######################################


# MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium V2_03_difficult

#dataname="V1_01_easy"
#dataname="V1_02_medium"
#dataname="V1_03_difficult"
dataname="V2_01_easy"
#dataname="V2_02_medium"
#dataname="V2_03_difficult"
#dataname="MH_01_easy"
#dataname="MH_02_easy"
#dataname="MH_03_medium"
#dataname="MH_04_difficult"
#dataname="MH_05_difficult"


# 计算ATE
Cur_Dir="/home/hri/rosbuild_ws/package_dir/ORB_SLAM2_OpenCV3.2/Examples/ROS/ORB_VIO/"
plotATE_autoScale=$dataname"_autoScale"
plotATE_estimatedScale=$dataname"_estimatedScale"

groundtruth="$Cur_Dir/GroundTruth/EuRoc/$dataname/groundtruth.txt"
trajectoryNavState="$Cur_Dir/KeyFrameNavStateTrajectory.txt"


iterateTime=1

# create resultDatasetFolder.

resultDatasetFolder="$Cur_Dir/results/EuRoc/$dataname/"
if [[ -d "$resultDatasetFolder" ]]; then
    rm -rf $resultDatasetFolder
fi
mkdir "$Cur_Dir/results/EuRoc/"
mkdir $resultDatasetFolder

# create two txt files to save the statistics information for auto scale and estimated scale
cd $resultDatasetFolder
statisticForAutoScaleTxT=$resultDatasetFolder"/statisticForAutoScaleTxT.txt"
touch $statisticForAutoScaleTxT
statisticForEstimatedScaleTxT=$resultDatasetFolder"/statisticForEstimatedScaleTxT.txt"
touch $statisticForEstimatedScaleTxT


# for iteration
for ite in $(seq 1 $iterateTime)
do
    echo ""
    echo "***************** dataname = ${dataname} ****************"
    echo "**************** iterate counter = ${ite} ****************"

    # run VI-ORB_SLAM
    roslaunch ORB_VIO testeuroc.launch sequence_name:="/home/hri/Documents/SLAM_Dataset/EuRoC/rosbag_file/$dataname.bag"

    # copy tmp_results and rename as $ite
    cp -r "$Cur_Dir/tmp_results" $resultDatasetFolder
    mv tmp_results $ite

    # copy estimated trajectory
    cp "$Cur_Dir/KeyFrameTrajectory.txt" $ite
    cp "$Cur_Dir/KeyFrameNavStateTrajectory.txt" $ite

    # 生成auto sacle后的图片，关闭曲线标注
    python ~/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_ate_autoScale_completerate.py --plot "$ite/$plotATE_autoScale" --verbose /$groundtruth  "$ite/KeyFrameNavStateTrajectory.txt" --save_evaluateResults $statisticForAutoScaleTxT --iterateCounter $ite

    # 生成estimated_scale后的图片，关闭曲线标注
    python ~/rgbd_benchmark_tools/src/rgbd_benchmark_tools/evaluate_ate_estimatedScale_completerate.py --plot "$ite/$plotATE_estimatedScale" --verbose /$groundtruth "$ite/KeyFrameNavStateTrajectory.txt" --save_evaluateResults $statisticForEstimatedScaleTxT --iterateCounter $ite

done


