^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ethernet_rmp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2014-07-30)
------------------
* Merge pull request #11 from cmdunkers/develop
  added in joint state publisher from the segway feedback for the wheels
* added the correct joint lint names
* fixed package name
* added a joint state publisher for the wheels based ont he feedback
* Contributors: Chris Dunkers, Russell Toris

0.0.2 (2014-07-29)
------------------
* Merge pull request #10 from cmdunkers/develop
  fixed the yaw rates for carl
* fixed the yaw rates for carl
* Contributors: Chris Dunkers, Russell Toris

0.0.1 (2014-07-23)
------------------
* Merge pull request `#9 <https://github.com/WPI-RAIL/ros_ethernet_rmp/issues/9>`_ from cmdunkers/develop
  Added a param to publish or not to publish the odometry transform for th...
* Added a param to publish or not to publish the odometry transform for the pose update
* Merge pull request `#8 <https://github.com/WPI-RAIL/ros_ethernet_rmp/issues/8>`_ from cmdunkers/develop
  fixed rotation for the segway from RHR to +ve is clockwise
* fixed rotation for the segway from RHR to +ve is clockwise
* Merge pull request `#7 <https://github.com/WPI-RAIL/ros_ethernet_rmp/issues/7>`_ from cmdunkers/develop
  made odom reference base_footprint
* changed the odom to be with respect to the base_footprint
* Merge pull request `#2 <https://github.com/WPI-RAIL/ros_ethernet_rmp/issues/2>`_ from WPI-RAIL/develop
  renamed functions to underscores
* rename functions to underscores
* Merge pull request `#6 <https://github.com/WPI-RAIL/ros_ethernet_rmp/issues/6>`_ from cmdunkers/develop
  Fixed the config parameters bug
* Fixed the issue of the user defined parameters not being set
* Found issue with config upload
* Merge pull request `#5 <https://github.com/WPI-RAIL/ros_ethernet_rmp/issues/5>`_ from cmdunkers/develop
  cleaned up the ethernet_rmp and pose updater files
* fixed up the large if else statement and cleaned up the pose updater
* Merge branch 'develop' of https://github.com/WPI-RAIL/ros_ethernet_rmp into develop
  Conflicts:
  AUTHORS.md
* more cleanup
* cleanup
* cleanup of pose stuff
* renamed some things
* message cleanup
* cleanup and travis
* fixed authors
* Merge pull request `#1 <https://github.com/WPI-RAIL/ros_ethernet_rmp/issues/1>`_ from cmdunkers/develop
  Added initial ROS wrapper nodes
* added git ignore
* cleanup
* Basic Cleanup
* fixed params issues
* made a check for the parameters and made them all rosparams
* Fixed a few bugs and defined thermpCommand message
* added comd_vel and RMP_COMMAND
* worte and modified the ROS side of the segway interface
* Initial commit
* Contributors: Chris Dunkers, Russell Toris, cmdunkers
