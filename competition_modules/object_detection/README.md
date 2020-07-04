# opencv_object_detection
Object detection with opencv.

1. Download the bag file from
	https://drive.google.com/open?id=12d6RssFZCNQ9QS5jQRjoXHv8IbrBTX5a

2. clone this package
```
cd ~/<your workspace>/src
git clone https://github.com/ChengChengYang1997/opencv_object_detection.git
cd ~/<your workspace>
```

3. Play the bag file
```
roscore
cd ~/<the directory of bag file>
rosbag play lab5_bag.bag -l
```

4. Run the node and rviz
```
rosrun opencv_object_detection object_detection
cd ~/<the directiory of rviz>
rviz -d rviz_object_detection
```
