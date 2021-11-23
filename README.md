# Aruco Pose Filter

Aruco Pose Filter ROS2 Package.

## Description

ROS2 Package to filter the pose of an aruco.
Position (x, y, z) filtered with a butterworth filter of 3rd order, with sampling rate and cutoff frequency configurable via params.
Orientation is filtered with a slerp filter: for the current measure, it takes the last n samples and apply a slerp to compute the mean quaternion.

## Input/Output

Input: 

- Pose: 				aruco_pose_raw_measures_prefix + marker_id 		(geometry msgs/Transform Stamped)
- Presence: 			aruco_presence_prefix + marker_id				(std msgs/Bool)

Output: 

- Filtered Pose: 		aruco_pose_filter_measures_prefix + marker_id 	(geometry msgs/Transform Stamped)
						published in camera frame (passed as param)

## TF2

The node also publish a transform from camera frame to (stable_link_prefix + marker_id)

## Depend

ROS2
Eigen