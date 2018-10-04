# ROS package for TeraRanger modules

 This package is a collection of nodes for TeraRanger single sensor modules.

 * [TeraRanger Evo 64px](https://www.terabee.com/portfolio-item/teraranger-evo-64px-multi-pixel-time-of-flight-sensor/)
 * [TeraRanger Evo 60m](https://www.terabee.com/portfolio-item/teraranger-evo-infrared-distance-sensor/)
 * [TeraRanger Evo 600Hz](https://www.terabee.com/portfolio-item/teraranger-evo-600hz/)
 * [TeraRanger Evo 3m](https://www.terabee.com/portfolio-item/teraranger-evo-3m/)
 * [TeraRanger One](https://www.terabee.com/portfolio-item/teraranger-one/)
 * [TeraRanger Duo](https://www.terabee.com/portfolio-item/teraranger-duo/)

Link to ROS Wiki package documentation: [Here](http://wiki.ros.org/teraranger)

## Dependencies
This package depends on this [serial](http://wiki.ros.org/serial) library. To get it, execute the following command:

```
sudo apt-get install ros-<your_distro>-serial
```

where <your_distro> is your ROS distribution (e.g. kinetic, lunar, indigo).

If it's not available for your distribution, clone https://github.com/wjwwood/serial into your workspace, then build and source your workspace.

## Installing the package

### Installing from PPAs

You can install the teraranger package by running the command:

```
sudo apt-get install ros-kinetic-teraranger
```

### Building and Running the package from source

 To clone and build the package in your workspace follow these steps:

* If you have ssh key setup for your github account:

```
cd ~/ros_ws/src
git clone git@github.com:Terabee/teraranger.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

* If you prefer to use https use this set of commands:

```
cd ~/ros_ws/src
git clone https://github.com/Terabee/teraranger.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## Running the TeraRanger Evo 64px

After your workspace is built and sourced:
```
rosrun teraranger evo_64px.py _portname:=/dev/ttyACM0
```

This node is publishing on two topics:
* teraranger_evo_64px/depth_image: a colormapped RGB image based on depth data
* teraranger_evo_64px/point_cloud: a point cloud in the frame of the sensor

## Running the TeraRanger Evo 60m

After your workspace is built and sourced:
```
rosrun teraranger evo _portname:=/dev/ttyACM0 _sensor_type:=Evo_60m
```

## Running the TeraRanger Evo 600Hz

After your workspace is built and sourced:
```
rosrun teraranger evo _portname:=/dev/ttyACM0 _sensor_type:=Evo_600Hz
```

## Running the TeraRanger Evo 3m

After your workspace is built and sourced:
```
rosrun teraranger evo _portname:=/dev/ttyACM0 _sensor_type:=Evo_3m
```

**WARNING: By default, if no sensor_type is specified, the default sensor chosen is the Evo 60m**

## Running the TeraRanger One

After your workspace is built and sourced:
```
rosrun teraranger one _portname:=/dev/ttyACM0
```

## Running the TeraRanger Duo

After your workspace is built and sourced:
```
rosrun teraranger duo _portname:=/dev/ttyACM0
```

## Changing Sensor Parameters

You can change the mode of the sensors by running **rqt_reconfigure**:

```
rosrun rqt_reconfigure rqt_reconfigure
```

## Displaying Sensor Information

When the teraranger node is running in a new terminal window execute:

```
rostopic list
```

to see list of available topics. If the teraranger node is running and the sensor is connected to your PC you should see topics starting with /teraranger.

To display the messages arriving on the topic run the following command in a terminal:

```
rostopic echo /teraranger_<sensor_name>
```

where <sensor_name> is the name of your sensor (e.g. one, evo).

## Product pictures and where to get the sensors

### TeraRanger Evo 64px

<img src="https://www.terabee.com/wp-content/uploads/2018/09/Evo-64px-no-background-1.png" width="300"/>

| Information |
| -------------- |
|[Product page Evo 64px](https://www.terabee.com/portfolio-item/teraranger-evo-64px-multi-pixel-time-of-flight-sensor/)|
|[Specification sheet](https://www.terabee.com/wp-content/uploads/2018/10/TeraRanger-Evo-64px-Specification-sheet.pdf)|
|[Online shop](http://www.teraranger.com/product/teraranger-evo/)|

### TeraRanger Evo 3m

<img src="https://www.terabee.com/wp-content/uploads/2018/08/TeraRanger-Evo-3m-bottom-picture-450px.jpg" width="300"/>

| Information |
| -------------- |
|[Product page Evo 3m](https://www.terabee.com/portfolio-item/teraranger-evo-3m/)|
|[Specification sheet](http://www.terabee.com/wp-content/uploads/2018/09/TeraRanger-Evo-3m-Specification-sheet.pdf)|
|[Online shop](http://www.teraranger.com/product/teraranger-evo/)|

### TeraRanger Evo 60m/Evo 600Hz

<img src="http://www.teraranger.com/wp-content/uploads/2017/04/DSC0977-Editar.jpg" width="300"/>

| Information |
| -------------- |
|[Product page Evo 60m](https://www.terabee.com/portfolio-item/teraranger-evo-infrared-distance-sensor/) / [Product page Evo 600Hz](https://www.terabee.com/portfolio-item/teraranger-evo-600hz/)|
|[Specification sheet](https://www.terabee.com/portfolio-item/teraranger-evo-infrared-distance-sensor/#specifications)|
|[Online shop](http://www.teraranger.com/product/teraranger-evo/)|

### TeraRanger One

<img src="http://www.teraranger.com/wp-content/uploads/2016/04/TeraRanger-Frame-Side-View.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/portfolio-item/teraranger-one/)|
|[Specification sheet](https://www.terabee.com/portfolio-item/teraranger-one/#teraranger-specifications)|
|[Online shop](http://www.teraranger.com/product-category/sensors/) |

### TeraRanger Duo

<img src="http://www.teraranger.com/wp-content/uploads/2014/06/TRduo.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](https://www.terabee.com/portfolio-item/teraranger-duo/)|
|[Specification sheet](https://www.terabee.com/portfolio-item/teraranger-duo/#teraranger-specifications)|
|[Online shop](http://www.teraranger.com/product-category/sensors/) |
