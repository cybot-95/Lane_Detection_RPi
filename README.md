# Lane Detection and Lane Keep Assist Using Raspberry Pi
	
# Description:
  -----------

This Project is all about avoiding the vehicle deviating from the current lane. 
It uses camera equipped on board to detetct any variations of lane in upcoming patch of road.

# Thesis of Lane Keeping Algorithm:
 --------------------------------
Python Version: 3.9.6
pip version: 1.18.4

The algorithm starts with importing few modules namely, OpenCV,NumPy, and RPi.GPIO.

1.OpenCV: OpenCV is Computer Vision Library Tool.In this project we use OpenCV to create a region of interest to detect while lane markings 
of the selected region.

2.NumPy: NumPy is a Python Library used for working with arrays.It consist of functions for Linear Algebra,Matrices, and Fourier Transforms etc.
In this project we use NumPy to differentiate lane segment of road using functions like Slope Intercept by converting captured image into a pixel matrix.

3.RPi.GPIO: GPIO is a python module for enabling General Input Output Pins present on the Pi Board. 

> After installing Modules we initialize and setup GPIO pins for controlling DC motor speed and direction. A user defined functions "detect_edge" enables camera feed and filters part of road with white color using HSV Values.
Along with that Canny function of OpenCVis used to filter out edges of lane.

> Next user defined function named "region_of_interest" crops the image frame into half for selecting region of lanes.

>Next user defined function named "detect_line_segments" detects the line segment using Hough Transformation.

>Next another user defined function named "average_slope" is used to calculate slope of lane lines and bifurgate them into left and right lanes.

>Another user defined function named "make_points" creates points on the selcted region of intrest for plotting lane markings.

>A user defined function named "display_lines" plots lines for lane marking on points created by previous funtion.

>Steering angle is achieved by another user defined function named "get_steering_angle".It calculates lane line coordinates offsets and obtains angle.Later 
obtained angle is converted from radian to degrees. I

>Finally the code starts a while loop where using if conditions for deviation found out and vehicle is steered according to it.

# Components Used:
 ---------------

1.Raspberry Pi Zero 2w > 
512MB Ram.
64-bit Arm Processor @ 1GHz.
40 Pins.
Mini HDMI port *1, Micro USB port *2.
CSI-2 Camera Connetor.
Operates at 5v/2A.
						 
2.Pi Camera Module > 
5 megapixel Omnivision 5647 Sensor
2592 x 1944 Image Resolution.
1080p @ 30fps / 720p @ 60fps / 480p @ 90fps Video Resolution.
15 pins CSI (Camera Serial Interface).
						 
3.L298N Motor Driver > 
H-Bridge Circuit Controller.
5V-35V/2A Motor Output Voltage.
5V-7V/2A  Logic Input Voltage.
Capable of Handling 2 DC motors.
PWM(Pulse Width Modulation) Enabled for Speed Control.
						 
3.RC Car > 
Dual DC motor.
Operates at 5v/2A.
						 
4.Battery > 
Li-ion Powerbank.
5000mAh Capable of 4+ Hrs Backup.
5V/2A Output @ 2 USB Ports.


# Software Used:
 -------------

1.Pi Operating System > 
Raspbian Pi Legacy OS 64-bit. 
Kernel Version: 5.10.
Debian Version: 10.
						
2.Windows Operating System > 
Windows 11 Home
Version: 2H22
Processor: Intel i5-11320H 64-bit @ 2.5Ghz
RAM: 8GB DDR4 @ 3200MHz
							 

# Note: 
You can use Raspberry Pi Zero or greater board. RPi 3b is suggest for non laggy experience.

						 
