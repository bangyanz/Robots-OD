FILE(REMOVE_RECURSE
  "../src/robot_pose_ekf/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/robot_pose_ekf/srv/__init__.py"
  "../src/robot_pose_ekf/srv/_GetStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
