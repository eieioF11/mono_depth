#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ${ROS_WORKSPACE}/install/local_setup.bash;

models=(
  "vits"
  "vitb"
  "vitl"
)

for MODEL in "${models[@]}" ; do
  echo "Running export_v2 for $MODEL"
  ros2 run mono_depth export_v2 --encoder $MODEL
  IN_PATH=models/depth_anything_v2_$MODEL.onnx
  OUT_PATH=models/depth_anything_v2_$MODEL.engine
  echo "Exporting $IN_PATH to $OUT_PATH"
  /usr/src/tensorrt/bin/trtexec --onnx=$IN_PATH --saveEngine=$OUT_PATH
done
