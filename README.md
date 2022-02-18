Real-time_robot_control
-----------------------
  This is a software to demonstrate the method of controlling an industrial robot in real time using an accelerometer and gyroscope.
  This device is basically attached to a special glove, which is worn by the employee. The device operates wirelessly.
  The PDF file contains scientific work that describes the operation of this device.
  A graphical interface has also been created to demonstrate the operation of this method.

Hardware components
--------------------   
1. Arduino Uno
2. Gyroscope and accelerometer GY-85
3. Pair of radio transmitter / receiver nRF2401

Software components
-------------------
1. main: Removal of indicators from sensors, data processing, sending of data to receiving base.
2. Data_transmit: Receiving data and transmitting via com port to a graphical environment.
3. Visualization: 3d visualization of the robot control process.

![3d_visualization](https://user-images.githubusercontent.com/75342698/154772679-87f84a44-2937-4ff7-ab6a-c3c54770e5a9.png)
