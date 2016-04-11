Requires running tf linking package gazebo_tf_linker

Usage: rosrun vr_track_alvar individual [Marker Size (cm)] [Output Frame] [List of Markers as " 'parent/robot name'/'marker joint name':'marker number' "
EG: rosrun vr_track_alvar individual 5.5 head_mount_kinect_ir_optical_frame workbench/marker6:6 workbench/marker7:7

Standard model included with appropriate launch file