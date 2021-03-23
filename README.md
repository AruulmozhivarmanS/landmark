# Landmark based Navigation using AprilTags
- This section describes the steps to run the tag-based localization code.

## Dependencies
- ros-kinetic/ros-melodic
- apriltag
- apriltag_ros

## Steps to Run
1. ````roslaunch apriltag_ros continous_detection.launch````
2. ````python publish_tf.py````
3. ````python correction.py````

## Description of relevant scripts
#### publish_tf.py
- The main script that is responsible for launching the tag-based localization node.
- Here you can specify the file to read the set of tag locations from.
- The tag locations are specified in the form of a text file. Each text file contains the 6DoF pose of each tag wrt a reference.
- Check [example](config/rbc_gt.txt)

#### tf_class.py
- This has the class for handling the tag locations and also determining the location of the robot whenever a tag is detected.
- The library used mainly is TF, which is a package used by ROS to keep track of different reference frames in the environment.
- Here each tag is added to a common reference frame named ````tag_world_ref````in the TF-tree.
- Whenever a tag is visible, the transform between ````tag_world_ref```` and the ````robot (odom or base_link)```` is updated.
- NOTE: The first time a tag is seen is when the above mentioned steps begin. Until then the script waits for a detection.

#### correction.py and check.py
- This script is responsible for publishing various topics related to the tag-based localization.
1. ````tag_map/pose```` - This topic pubishes the pose of the robot in the tag map.
2. The same topic ````tag_map/pose```` can also publish the pose of the robot in the form of odometry. (Set ````full_correction```` to True)
3. The difference between 1. and 2. is:
   - (1) gives the instantanous pose of the robot ONLY when the tag is visible.
   - (2) gives a continous trajectory of the robot. When a tag is visible, the odometry is corrrected according to the pose given by the tag, otherwise the  odometry is used
4. ````tag_map/markers```` is a topic that published the pose of each tag in the map.

# Optimization of Tag Locations
- This section describes the data collection process and the optimization process once the tag locations have been measured manually.

## Dependencies
- scipy
- Rest are same as above

## Data Collection
- First we run the tag based localization node as decribed above.
  - NOTE: Run ````check.py```` instead of ````correction.py```` in step 3. above.
- Then, the following topics need to be collected as bag files for the optimization process.
  - ````/tag_map/pose_pc````
  - ````/tag_detections````, from the ````apriltag_ros```` node.
  - ````/odometry/filtered````, the odometry topic of the robot.
  - ````/tag_map/odom_filtered_pc````, the odometry topic in the tag reference frame.
  - ````/tag_map/markers_pc````, the locations of the tags.
- After collecting the bag file, run ````sync_npy.py````.
  - This is the preprocessing step. Here the topic are time synchronized and saved as ````.npy```` files.
  - The location of the folder and the config file to use can be specified here.
  - NOTE: Please run the collected bag file AFTER running the above script.
- ````rosbag play <collected_bag>.bag && rosbag play done.bag````. Please use the following command when synchronozing the data.

## Optimization
- There are two different ways for doing the optimization. The differences lie in the optimization objective.
  1. ````Relative distances (optimize_pose_dist.py)```` - Here the objective is to minimize error in distance between a pair of points measured by the GT and the Tag based localization.
  2. ````Absolute distances (optimize_pose.py)```` - Here the objective is to minimize error between the GT point and the correcponding Tag based localization point.
- In the scripts you need to add the directory that contains the data collected. The structure of the folder should be the following:
  - <main_folder name>
    - <run_number>_npy
      - gt
        - <tag_id>.npy
      - data
        - <tag_id>.npy
- The script prints the error measure for each tag to keep track of the optimization progress.
  - NOTE: Sometimes the script may not terminate for a long time, in this case you can interrupt the script and the most recent iteration will be saved as the optimized result.
  - NOTE: Please change the name of the file mentioned in the ````write_file()```` function, whenever you are starting a fresh run.

## Some more info...
1. During data collection over multiple runs, you always need to start at the same tag. So it is better to fix that tag as origin (ie: (0, 0, 0), (0, 0, 0, 1))
2. The rest of the tags can be measure with respect to this tag, while physically measuring.
3. This is very important, otherwise the optimization will fail or result in absurd values.
4. One common issue with apriltags is axis-flipping. When the surface normal of the tag is aligned to the normal of the camera, there is an ambiguity in the pose. The ambiguity results in two possible poses for the tag. In these cases it is better to reject these pose estimates as it would disrupt the optimization process.
   - A simple way to do this would be to check if the robot is upright or upside down based on the tag's pose estimate. (Not been implemented, just a suggestion)
   - Another way to rectify this would be to add multiple tags on the same sheet of paper (tag bundles). Here PNP would use all the tags in the sheet of paper to estimate the pose. This reduces the number of axis flippings.
   - For now, during optimization if the error is above some threshold say (2 meters) then that data point is rejected.
