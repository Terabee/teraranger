# ROS package for TeraRanger modules

 This package is a collection of nodes for TeraRanger single point sensor modules.

 * [TeraRanger Evo 60m](https://www.terabee.com/portfolio-item/teraranger-evo-infrared-distance-sensor/)
 * [TeraRanger Evo 600Hz](https://www.terabee.com/portfolio-item/teraranger-evo-600hz/)
 * [TeraRanger One](https://www.terabee.com/portfolio-item/teraranger-one/)
 * [TeraRanger Duo](https://www.terabee.com/portfolio-item/teraranger-duo/)

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

WARNING: By default, if no sensor_type is specified, the default sensor chosen is the Evo 60m

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

## Product pictures and where to get the sensors

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
