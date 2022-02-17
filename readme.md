# What?
ROS node implementation based on prototype @ https://github.com/stoddabr/python_unity_framestreamer

Built to stream images from a ROS topic to Unity but could be used elseware.

# How?

1. Do the ros stuff: git clone, source, chmod python files, and catkin make/build, and source again to be safe
2. Open two terminals
3. In the first either run `roslaunch tcp_img_streamer trisect.launch` or `roslaunch tcp_img_streamer multitrisect.launch`(if you want to stream to multiple objects in a scene or multiple Unity instances*)
4. In the second run `rosbag play -l tcp_img_streamer/bag/trisect_emulator_sample.bag` to get run a bag file containing a video stream to topic `/color/image_raw`
5. Connect to Unity using the scripts in this repo https://github.com/stoddabr/python_unity_framestreamer

\*streaming to multiple scenes at a time impacts frame rate 

# Known issues

* The bag file is large, should get a smaller one
* For some reason, bag file is not working properly when started within launch file
* fps is low. Like 1-5fps when running from a docker container to a windows machine. Drops to below 1fps with three streamers. ~90% of this time is sending data over a socket
* jpeg encoding is not efficient for video, for example h264 would be much more efficient 
* only one stream is used to send data (ie its not [muxed](https://en.wikipedia.org/wiki/Multiplexing))
* I am not an expert in writing socket code so there are some edge cases that you may run into

# TODO

* Create basic launch file withoug the trisect bloat
* Create test/demo launch file from bag https://answers.ros.org/question/62811/how-to-play-rosbag-using-launch-file/
* Multiple threads https://github.com/stoddabr/python_unity_framestreamer/tree/multiple_connections
