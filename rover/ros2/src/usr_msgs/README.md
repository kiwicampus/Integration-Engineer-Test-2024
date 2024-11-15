# User messages

Package in charge of allocating the ROS2 user-defined messages. 

All the messages defined in this package are grouped by the name of the packages inside our development workspace, to enhance readability and clarity of where will be used defined messages.

## Message addition

To add new messages, please make sure to create the folder with the same name as the package where the message will be used. The message name should fulfill the ROS2 standards (In terms of case style) which are: CustomMessage.msg

Once the message file is created, add it to the `MSG_FILES` section inside the `CMakeLists.txt`, ROS2 messages used inside your message definition should be linked into this same at `find_package()`

```
File Tree
📦usr_msgs
 ┣ 📂msg
 ┃ ┣ 📂fail_detection
 ┃ ┃ ┣ 📜Fail.msg
 ┃ ┃ ┗ 📜Fails.msg
 ┃ ┣ 📂location
 ┃ ┃ ┗ 📜LocationMsg.msg
 ┃ ┣ 📂smart_brake
 ┃ ┃ ┗ 📜EmergencyStop.msg
 ┃ ┣ 📂tf_mini
 ┃ ┃ ┣ 📜BrakeStatus.msg
 ┃ ┃ ┣ 📜Sensor.msg
 ┃ ┃ ┗ 📜Sensors.msg
 ┣ 📜CMakeLists.txt
 ┣ 📜README.md
 ┗ 📜package.xml

```
<!-- ----------------------------------------------------------------------------------------------------------------------- -->
## Messages

<!-- ----------------------------------------------------------------------------------------------------------------------- -->
### Fail Detection

Messages in charge of notifying fail events.

* **Fail.msg** Message containing the information about a fail event.
    * event      [string]: A string describing the event.
    * confidence [uint8] : On almost all the cases define the ID of the action to be executed.

<!-- ----------------------------------------------------------------------------------------------------------------------- -->
### Location

*  **LocationMsg.msg:** Message containing the measurements for the IMU and GPS sensor
    * header    [std_msgs/Header]: standard header message
    * heading   [uint16]         : robot orientation
    * status    [int8]           : fixed status
    * roll      [float32]        : roll measure by IMU
    * pitch     [float32]        : pitch measure by IMU
    * yaw       [float32]        : yaw measure by IMU
    * latitude  [string]         : given latitude provided by the current GNSS device
    * longitude [string]         : given longitude provided by the current GNSS device 
    * accuracy  [float32]        : accuracy calculated by the covariance provided by GNSS device
    * sat       [int8]           : number of satellites available for the GNSS device
    * sensor    [string]         : GNSS source of information GPS/WIFI


### Smart Brake

Messages in charge of reporting the source and data of the smart_brake.
*  **EmergencyStop.msg:** Current status of the smart_brake node
    * data [bool] : Enabled/Disabled smart_brake by range sensors source
    * src  [uint8]: Enabled/Disabled smart_brake by cliff sensors 

### Tf_Mini

Messages are in charge of controlling the devices embedded in the system (physical or virtual).
*  **BrakeStatus.msg:** Current status of the smart_brake node
    * range_sensors [int8]: Enabled/Disabled smart_brake by range sensors source
    * cliff_sensors [int8]: Enabled/Disabled smart_brake by cliff sensors
    Convention:
        - Unknown: -1
        - Automatically disabled: 0
        - Functional: 1
        - Disabled by detection: 2
        - Disabled by supervisor: 3
        - Reading error: 4
        - Disabled by the map: 5

*  **Sensors.msg:** Report the current state of the distance sensors (connected, disconnected or never seeing)
    * sensors [Sensor[8]]: Message to report the state, channel and type of sensor
    * mux     [int8]    : 0 never connected, 1 connected and -1 disconnected

*  **Sensor.msg:** Message to report the state, channel and type of sensor
    * state   [int8]  : 0 never connected, 1 connected and -1 disconnected
    * channel [uint8] : The channel in which the sensor is connected (0 to 7)
    * type    [string]: can be "cliff" or "distance"
