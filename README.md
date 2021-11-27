# Ros-ESP8266-node
Files for ROS to ESP8266 microcontroller connection. ESP8266 is a popular microcontroller for IOT applications where as ROS is a software for robot programming. Bridging both fields would pave way for new applications

<h2> Requirements</h2>
<h6> ESP8266/ ESP32 microcontroller
<h6> ROS kinetic or noetic
  
<h2> Installation
  Install ROS,ROS server, rosserial, gather the IP of the server
  Configure WiFi parameters on EspRos code, to allow microcontroller to connect to Ros server. 
  After that, all standard ROS commands can be used. 
  To use rostopic commands, make sure the topics are coded into the microcontroller. That can be done by modifying .ino file
