# ROS package for TeraRanger module

 This package is a collection of nodes for Teraranger module single sensor solutions:
 * [Teraranger One](http://teraranger.com/portfolio-item/teraranger-one/)
 * [Teraranger Duo](http://teraranger.com/portfolio-item/teraranger-duo/)

## Building and Running the package from source

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

## Running the Teraranger One

After your workspace is built and sourced:
```
rosrun teraranger one _portname:=/dev/ttyACM0
```

## Running the Teraranger Duo

After your workspace is built and sourced:
```
rosrun teraranger duo _portname:=/dev/ttyACM0
```

## Changing Sensor Parameters

You can change the mode of the sensors by running **rqt_reconfigure**:

```
rosrun rqt_reconfigure rqt_reconfigure
```

## Product pictures and where to get the sensors

### Teraranger One

<img src="http://www.teraranger.com/wp-content/uploads/2016/04/TeraRanger-Frame-Side-View.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](http://teraranger.com/portfolio-item/teraranger-one/)|
|[Specification sheet](http://teraranger.com/portfolio-item/teraranger-one/#teraranger-specifications)|
|[Online shop](http://www.teraranger.com/product-category/sensors/) |

### Teraranger Duo

<img src="http://www.teraranger.com/wp-content/uploads/2014/06/TRduo.jpg" width="300"/>

| Information |
| -------------- |
|[Product page](http://teraranger.com/portfolio-item/teraranger-duo/)|
|[Specification sheet](http://teraranger.com/portfolio-item/teraranger-duo/#teraranger-specifications)|
|[Online shop](http://www.teraranger.com/product-category/sensors/) |
