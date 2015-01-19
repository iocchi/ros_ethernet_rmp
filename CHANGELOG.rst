^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_ethernet_rmp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2015-01-19)
------------------
* Added approximation of caster wheel orientaiton
* Contributors: David Kent

0.0.6 (2014-09-02)
------------------
* fixed wheel direction for joint state publisher to work correctly with the fixed urdf
* Contributors: dekent

0.0.5 (2014-08-15)
------------------
* Merge pull request #17 from cmdunkers/develop
  updated yaw accel limit
* updated the yaw_accel_limit
* Merge branch 'develop' of https://github.com/cmdunkers/ros_ethernet_rmp into develop
* Merge pull request #2 from WPI-RAIL/develop
  cmdunkers
* Merge pull request #1 from WPI-RAIL/develop
  updates
* fixed joint state publisher issue
* Contributors: Chris Dunkers, Russell Toris, cmdunkers

0.0.4 (2014-08-05)
------------------
* fixed looking for wrong packages
* travis now pull messages from source
* moved messages to rmp_msgs
* Merge pull request #16 from cmdunkers/develop
  changed to rmp_msgs
* changed to rmp_msgs
* Battery monitor added to travis
* fixed CMakeLists bug
* Merge pull request #15 from cmdunkers/develop
  direction!
* direction!
* Merge pull request #14 from cmdunkers/develop
  radians!
* radians!
* missing thing
* launch file fix
* Merge pull request #12 from cmdunkers/develop
  Fixed issues with the wheel joint state
* fixed the modulo
* Fixed issues with the wheel joint state
  added the battery monitor to the launch file
* Contributors: Chris Dunkers, David Kent, Russell Toris, dekent

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
