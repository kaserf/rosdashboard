ROS Dashboard
=============

Authors
-----
* Felix Kaser <f.kaser@gmx.net>

Overview
--------
ROS Dashboard is a graphical tool to allow you easy debugging and monitoring of your robot. It is designed to give you all the flexibility to extend the tool however you want to fit your scenario best.

The dashboard can be populated with widgets which can be hooked up with ROS topics. It allows you to have a simple visual representation of the data you pipeline through ROS topics. The original idea for the project was to use it for debugging. Having a visual representation of the data can help you to find bugs faster. The design allows you to easily modify your dashboard if you have new data coming in.

The API allows you to publish data easily in your code. It is designed to be as easy to use as printf statements but with the addition to have type information for the data so that we can visualize the data in a better way than just a stream of text running down your console.

Dashboard
-------

The dashboard is a canvas where you can put your widgets on. The widgets are connected to ROS topics through the publish/subscribe mechanism. You can easily configure to which topic you want to subscribe and don't have to specify the message type at all, we take care of that.

API
-------
For an up to date version of the API please have a look at src/rosdashboard/api.py

The API is just a simple wrapper to allow one-line publishing of data in your source code. Internally we create a rospy.Publisher() and publish the data to the topic "/rosdashboard/<TAG>", with TAG being the first argument in one of the `log` calls of the API.

### Current API
* `rosdashboard.log(TAG, DATA)`
The type of data will be detected automatically if possible
* `rosdashboard.logint(TAG, VALUE, [MESSAGE_TYPE])`
Log an integer value. The default message type class is `std_msgs/Int32`
* `rosdashboard.logfloat(TAG, VALUE, [MESSAGE_TYPE])`
Log a float value. The default message type class is `std_msgs/Float32`
* `rosdashboard.logstring(TAG, MESSAGE)`
Log a string message. The message type is `std_msgs/String`
* `rosdashboard.logdata(TAG, DATA, MESSAGE_TYPE)`
Log arbitrary data. This allows you to create more sophisticated messages types and publish time just like you would with a normal rospy.Publisher()


Related Tools
---------
If you need more sophisticated visualization of your data there is RViz, which can visualize point clouds and other 3d data in a virtual environment. For datastream analysis there is rxplot, which draws a 2d plot of the data of a specific topic. ROS Dashboard lives somewhere in between, visualizing the data as you would like it to be visualized.

Examples
------
Imagine you are working on an algorithm to develop a line follower robot, where you have all the raw data from sensors and try to compute whether your robot should go straight, left or right. If you have some errors in your computation you will most likely end up printing the input values and some intermediary results to the console and trying to find out where your computation fails. With ROS Dashboard you can hook up your data streams to graphical widgets and get a more natural representation of the data (e.g. a horizontal bar telling you whether the robot wants to turn left, right or go straight).

Next steps
------------
* add plugin functionality to add more widgets
* integrate into ROS GUI / rqt
* allow resizing of widgets
* add save/restore functionality

Contributing
---------
If you want to help, just fork this repository and file pull requests. I would love to get as much feedback as possible and improve the tool further.
