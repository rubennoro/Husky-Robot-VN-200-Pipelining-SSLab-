Steps: 

Setup- 

EasyCAT Configuration (If not properly flashed): 

Git clone the EasyCAT folder for configuration and testing 

For getting xml file, open the configurator GUI, click project, and open the github repository .prj file 

Click create files to generate new .h, .prj, and .xml  

Then click write EEPROM, and open the bin file in the repo 

Other Steps- 

To power on Arduino Giga, connect it to Jetson Nano  

Connect Pins of VN-200 IMU to EasyCAT shield 

5V to Red 

GND to Black 

TX to Purple 

RX to Orange 

Place EasyCAT shield on top of Arduino Giga 

With Ethernet cable, connect EasyCAT shield to Speedgoat 

(Optional) If testing on computer, connect ethernet cable to computer instead 

Testing (ROS1)- 

Load the m4process and t3pipeline program code onto the Arduino Giga 

Go to tools and split flash 1 mB m7 and m4 each  

In m4process, on tools, set processor to M4 co-processor 

In t3pipeline, on tools, set processor to main processor 

Flash t3pipeline code first 

Open 3 terminals 

Run Roscore on Jetson Nano in one terminal 

Specific Commands 

Cd husky1_ws 

Source devel/setup.bash 

Rosbag play opti_test.bag in one terminal 

Rosrun opti_test receive_test.py immediately after 2c, in one terminal 

This will send 11 floats in the form of a byte array, 7 from Opti-track, 4 dummy data from RealSense 

IMU data will be asynchronously collected with M4 core of Arduino Giga  

Optional Debugging: 

If encountering issues with buffering or data inaccuracy, reset the Arduino Giga 

Open new terminal 

Cd arduinointerface/opti_read 

Python3 receive_test.py 

This script will print out the values the Arduino giga is sending  

If on Computer (Ignore if using Speedgoat): 

Open EasyCAT Navigator immediately after running step 1 in testing section 

Clone EasyCAT Github files for Navigator application 

Scan for connections and press run once connection is established 

Check for changing values of each variable (with IMU first, then other values) 
