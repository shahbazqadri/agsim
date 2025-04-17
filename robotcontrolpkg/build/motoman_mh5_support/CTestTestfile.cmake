# CMake generated Testfile for 
# Source directory: /home/sawyercontrol/Desktop/robotcontrolpkg/src/motoman/src/motoman/motoman_mh5_support
# Build directory: /home/sawyercontrol/Desktop/robotcontrolpkg/build/motoman_mh5_support
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_motoman_mh5_support_roslaunch-check_test_launch_test.xml "/home/sawyercontrol/Desktop/robotcontrolpkg/build/motoman_mh5_support/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/sawyercontrol/Desktop/robotcontrolpkg/build/motoman_mh5_support/test_results/motoman_mh5_support/roslaunch-check_test_launch_test.xml.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/sawyercontrol/Desktop/robotcontrolpkg/build/motoman_mh5_support/test_results/motoman_mh5_support" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/sawyercontrol/Desktop/robotcontrolpkg/build/motoman_mh5_support/test_results/motoman_mh5_support/roslaunch-check_test_launch_test.xml.xml' '/home/sawyercontrol/Desktop/robotcontrolpkg/src/motoman/src/motoman/motoman_mh5_support/test/launch_test.xml' ")
subdirs(gtest)
