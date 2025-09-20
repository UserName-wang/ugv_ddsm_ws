ros2 pkg create --build-type ament_cmake gluon_arm_description \
  --dependencies robot_state_publisher joint_state_publisher xacro
  
cd /home/panda/study/ros/ugv_ddsm_ws && colcon build --packages-select gluon_arm_description


ros2 pkg create --build-type ament_cmake gluon_arm_bringup

cd /home/panda/study/ros/ugv_ddsm_ws && colcon build --packages-select gluon_arm_bringup gluon_arm_description

ros2 launch gluon_arm_bringup gluon_arm.launch.py
ros2 run gluon_arm_hardware_interface test_actuator

ros2 run gluon_arm_hardware_interface test_actuator
[DEBUG] Sending handshake command: 
Hex dump: EE 00 44 00 00 ED 
ASCII:  ..D...

Handshake successful
[DEBUG] Sending query actuators command: 
Hex dump: EE 00 02 00 00 ED 
ASCII:  ......

Received 74 bytes in total after waiting
[DEBUG] Raw response data: 
Hex dump: EE 01 02 00 04 02 BC FD 28 80 F6 ED EE 02 02 00 04 02 BC 3A 2B 92 C7 ED EE 03 02 00 04 02 BC AF 45 7C 7B ED EE 04 02 00 04 02 BC FD 29 41 36 ED EE 05 02 00 06 02 BC AF 5A 00 00 51 25 ED EE 06 02 00 04 02 BC AF 61 7C 60 ED 
ASCII:  ........(..........:+...........E|{.........)A6.........Z..Q%.........a|`.

Found actuator with ID: 1
Actuator 1 serial number: 45940008
Found actuator with ID: 2
Actuator 2 serial number: 45890091
Found actuator with ID: 3
Actuator 3 serial number: 45920069
Found actuator with ID: 4
Actuator 4 serial number: 45940009
Found actuator with ID: 5
Actuator 5 serial number: 45920090
Found actuator with ID: 6
Actuator 6 serial number: 45920097
Successfully connected to ECB at 192.168.1.30:2000
[INFO] [1758355075.780856626] [rclcpp]: Found 6 actuators
[INFO] [1758355075.780941066] [rclcpp]: Enabling actuator 1...
[DEBUG] Sending command (addr=0x01, cmd=0x2A): 
Hex dump: EE 01 2A 00 01 01 7E 80 ED 
ASCII:  ..*...~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 01 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355076.660565381] [rclcpp]: Enabling actuator 2...
[DEBUG] Sending command (addr=0x02, cmd=0x2A): 
Hex dump: EE 02 2A 00 01 01 7E 80 ED 
ASCII:  ..*...~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 02 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355077.540229471] [rclcpp]: Enabling actuator 3...
[DEBUG] Sending command (addr=0x03, cmd=0x2A): 
Hex dump: EE 03 2A 00 01 01 7E 80 ED 
ASCII:  ..*...~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 03 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355078.419813568] [rclcpp]: Enabling actuator 4...
[DEBUG] Sending command (addr=0x04, cmd=0x2A): 
Hex dump: EE 04 2A 00 01 01 7E 80 ED 
ASCII:  ..*...~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 04 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355079.299463065] [rclcpp]: Enabling actuator 5...
[DEBUG] Sending command (addr=0x05, cmd=0x2A): 
Hex dump: EE 05 2A 00 01 01 7E 80 ED 
ASCII:  ..*...~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 05 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355080.107124414] [rclcpp]: Enabling actuator 6...
[DEBUG] Sending command (addr=0x06, cmd=0x2A): 
Hex dump: EE 06 2A 00 01 01 7E 80 ED 
ASCII:  ..*...~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 06 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355080.987199816] [rclcpp]: Checking error status for actuator 1...
[DEBUG] Sending command (addr=0x01, cmd=0x10): 
Hex dump: EE 01 10 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x10
[DEBUG] Raw response data: 
Hex dump: EE 01 10 00 01 01 ED 
ASCII:  .......

[INFO] [1758355080.988257396] [rclcpp]: No errors detected for actuator 1
[INFO] [1758355080.988303846] [rclcpp]: Checking error status for actuator 2...
[DEBUG] Sending command (addr=0x02, cmd=0x10): 
Hex dump: EE 02 10 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x10
[DEBUG] Raw response data: 
Hex dump: EE 02 10 00 01 01 ED 
ASCII:  .......

[INFO] [1758355080.989398495] [rclcpp]: No errors detected for actuator 2
[INFO] [1758355080.989446753] [rclcpp]: Checking error status for actuator 3...
[DEBUG] Sending command (addr=0x03, cmd=0x10): 
Hex dump: EE 03 10 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x10
[DEBUG] Raw response data: 
Hex dump: EE 03 10 00 01 01 ED 
ASCII:  .......

[INFO] [1758355080.990179048] [rclcpp]: No errors detected for actuator 3
[INFO] [1758355080.990203840] [rclcpp]: Checking error status for actuator 4...
[DEBUG] Sending command (addr=0x04, cmd=0x10): 
Hex dump: EE 04 10 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x10
[DEBUG] Raw response data: 
Hex dump: EE 04 10 00 01 01 ED 
ASCII:  .......

[INFO] [1758355080.990946059] [rclcpp]: No errors detected for actuator 4
[INFO] [1758355080.990976790] [rclcpp]: Checking error status for actuator 5...
[DEBUG] Sending command (addr=0x05, cmd=0x10): 
Hex dump: EE 05 10 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x10
[DEBUG] Raw response data: 
Hex dump: EE 05 10 00 01 01 ED 
ASCII:  .......

[INFO] [1758355080.991650780] [rclcpp]: No errors detected for actuator 5
[INFO] [1758355080.991676665] [rclcpp]: Checking error status for actuator 6...
[DEBUG] Sending command (addr=0x06, cmd=0x10): 
Hex dump: EE 06 10 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x10
[DEBUG] Raw response data: 
Hex dump: EE 06 10 00 01 01 ED 
ASCII:  .......

[INFO] [1758355080.999935016] [rclcpp]: No errors detected for actuator 6
[INFO] [1758355080.999967178] [rclcpp]: Switching actuator 1 to current mode...
[DEBUG] Sending command (addr=0x01, cmd=0x11): 
Hex dump: EE 01 11 00 01 01 7E 80 ED 
ASCII:  ......~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x11
[DEBUG] Raw response data: 
Hex dump: EE 01 11 00 01 01 ED 
ASCII:  .......

[INFO] [1758355081.000674464] [rclcpp]: Switching actuator 2 to current mode...
[DEBUG] Sending command (addr=0x02, cmd=0x11): 
Hex dump: EE 02 11 00 01 01 7E 80 ED 
ASCII:  ......~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x11
[DEBUG] Raw response data: 
Hex dump: EE 02 11 00 01 01 ED 
ASCII:  .......

[INFO] [1758355081.001345876] [rclcpp]: Switching actuator 3 to current mode...
[DEBUG] Sending command (addr=0x03, cmd=0x11): 
Hex dump: EE 03 11 00 01 01 7E 80 ED 
ASCII:  ......~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x11
[DEBUG] Raw response data: 
Hex dump: EE 03 11 00 01 01 ED 
ASCII:  .......

[INFO] [1758355081.001999941] [rclcpp]: Switching actuator 4 to current mode...
[DEBUG] Sending command (addr=0x04, cmd=0x11): 
Hex dump: EE 04 11 00 01 01 7E 80 ED 
ASCII:  ......~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x11
[DEBUG] Raw response data: 
Hex dump: EE 04 11 00 01 01 ED 
ASCII:  .......

[INFO] [1758355081.002661756] [rclcpp]: Switching actuator 5 to current mode...
[DEBUG] Sending command (addr=0x05, cmd=0x11): 
Hex dump: EE 05 11 00 01 01 7E 80 ED 
ASCII:  ......~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x11
[DEBUG] Raw response data: 
Hex dump: EE 05 11 00 01 01 ED 
ASCII:  .......

[INFO] [1758355081.003331539] [rclcpp]: Switching actuator 6 to current mode...
[DEBUG] Sending command (addr=0x06, cmd=0x11): 
Hex dump: EE 06 11 00 01 01 7E 80 ED 
ASCII:  ......~..

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x11
[DEBUG] Raw response data: 
Hex dump: EE 06 11 00 01 01 ED 
ASCII:  .......

[INFO] [1758355081.004008761] [rclcpp]: Setting target current to 0A for actuator 1...
[DEBUG] Sending command (addr=0x01, cmd=0x0A): 
Hex dump: EE 01 0A 00 04 00 00 00 00 00 24 ED 
ASCII:  ..........$.

Sent 12 bytes
Received -1 bytes
[INFO] [1758355083.012632421] [rclcpp]: Setting target current to 0A for actuator 2...
[DEBUG] Sending command (addr=0x02, cmd=0x0A): 
Hex dump: EE 02 0A 00 04 00 00 00 00 00 24 ED 
ASCII:  ..........$.

Sent 12 bytes
Received -1 bytes
[INFO] [1758355085.060713883] [rclcpp]: Setting target current to 0A for actuator 3...
[DEBUG] Sending command (addr=0x03, cmd=0x0A): 
Hex dump: EE 03 0A 00 04 00 00 00 00 00 24 ED 
ASCII:  ..........$.

Sent 12 bytes
Received -1 bytes
[INFO] [1758355087.108724168] [rclcpp]: Setting target current to 0A for actuator 4...
[DEBUG] Sending command (addr=0x04, cmd=0x0A): 
Hex dump: EE 04 0A 00 04 00 00 00 00 00 24 ED 
ASCII:  ..........$.

Sent 12 bytes
Received -1 bytes
[INFO] [1758355089.156696795] [rclcpp]: Setting target current to 0A for actuator 5...
[DEBUG] Sending command (addr=0x05, cmd=0x0A): 
Hex dump: EE 05 0A 00 04 00 00 00 00 00 24 ED 
ASCII:  ..........$.

Sent 12 bytes
Received -1 bytes
[INFO] [1758355091.204643490] [rclcpp]: Setting target current to 0A for actuator 6...
[DEBUG] Sending command (addr=0x06, cmd=0x0A): 
Hex dump: EE 06 0A 00 04 00 00 00 00 00 24 ED 
ASCII:  ..........$.

Sent 12 bytes
Received -1 bytes
[INFO] [1758355093.252634374] [rclcpp]: Reading current position of actuator 1...
readCurrentPosition called for actuator 0x1
[DEBUG] Sending command (addr=0x01, cmd=0x06): 
Hex dump: EE 01 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 01 06 00 04 00 8C B8 00 B3 CF ED 
ASCII:  ............

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: 0.549683
[INFO] [1758355093.253686815] [rclcpp]: Actuator 1 position: 0.549683
[INFO] [1758355093.253714234] [rclcpp]: Reading current position of actuator 2...
readCurrentPosition called for actuator 0x2
[DEBUG] Sending command (addr=0x02, cmd=0x06): 
Hex dump: EE 02 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 02 06 00 04 FA 03 84 00 A3 FC ED 
ASCII:  ............

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: -5.98627
[INFO] [1758355093.254827376] [rclcpp]: Actuator 2 position: -5.986267
[INFO] [1758355093.254877928] [rclcpp]: Reading current position of actuator 3...
readCurrentPosition called for actuator 0x3
[DEBUG] Sending command (addr=0x03, cmd=0x06): 
Hex dump: EE 03 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 03 06 00 04 08 A4 88 00 25 A7 ED 
ASCII:  .........%..

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: 8.6427
[INFO] [1758355093.256017342] [rclcpp]: Actuator 3 position: 8.642700
[INFO] [1758355093.256066886] [rclcpp]: Reading current position of actuator 4...
readCurrentPosition called for actuator 0x4
[DEBUG] Sending command (addr=0x04, cmd=0x06): 
Hex dump: EE 04 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 04 06 00 04 FC 8B A0 00 38 5E ED 
ASCII:  .........8^.

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: -3.45459
[INFO] [1758355093.257217519] [rclcpp]: Actuator 4 position: -3.454590
[INFO] [1758355093.257259201] [rclcpp]: Reading current position of actuator 5...
readCurrentPosition called for actuator 0x5
[DEBUG] Sending command (addr=0x05, cmd=0x06): 
Hex dump: EE 05 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 05 06 00 04 FF F7 2C 00 9D 02 ED 
ASCII:  .......,....

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: -0.0344849
[INFO] [1758355093.258359926] [rclcpp]: Actuator 5 position: -0.034485
[INFO] [1758355093.258422159] [rclcpp]: Reading current position of actuator 6...
readCurrentPosition called for actuator 0x6
[DEBUG] Sending command (addr=0x06, cmd=0x06): 
Hex dump: EE 06 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 06 06 00 04 27 00 10 00 07 50 ED 
ASCII:  .....'....P.

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: 39.0002
[INFO] [1758355093.259325694] [rclcpp]: Actuator 6 position: 39.000244
[INFO] [1758355095.259611144] [rclcpp]: Reading position of actuator 1 after waiting...
readCurrentPosition called for actuator 0x1
[DEBUG] Sending command (addr=0x01, cmd=0x06): 
Hex dump: EE 01 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 01 06 00 04 00 8C B8 00 B3 CF ED 
ASCII:  ............

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: 0.549683
[INFO] [1758355095.260848179] [rclcpp]: Actuator 1 position: 0.549683
[INFO] [1758355095.260892417] [rclcpp]: Reading position of actuator 2 after waiting...
readCurrentPosition called for actuator 0x2
[DEBUG] Sending command (addr=0x02, cmd=0x06): 
Hex dump: EE 02 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 02 06 00 04 FA 03 84 00 A3 FC ED 
ASCII:  ............

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: -5.98627
[INFO] [1758355095.261766404] [rclcpp]: Actuator 2 position: -5.986267
[INFO] [1758355095.261795909] [rclcpp]: Reading position of actuator 3 after waiting...
readCurrentPosition called for actuator 0x3
[DEBUG] Sending command (addr=0x03, cmd=0x06): 
Hex dump: EE 03 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 03 06 00 04 08 A4 7C 00 63 67 ED 
ASCII:  .......|.cg.

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: 8.64252
[INFO] [1758355095.262835827] [rclcpp]: Actuator 3 position: 8.642517
[INFO] [1758355095.262880723] [rclcpp]: Reading position of actuator 4 after waiting...
readCurrentPosition called for actuator 0x4
[DEBUG] Sending command (addr=0x04, cmd=0x06): 
Hex dump: EE 04 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 04 06 00 04 FC 8B 94 00 2E 9E ED 
ASCII:  ............

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: -3.45477
[INFO] [1758355095.263700240] [rclcpp]: Actuator 4 position: -3.454773
[INFO] [1758355095.263723390] [rclcpp]: Reading position of actuator 5 after waiting...
readCurrentPosition called for actuator 0x5
[DEBUG] Sending command (addr=0x05, cmd=0x06): 
Hex dump: EE 05 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 05 06 00 04 FF F7 34 00 97 02 ED 
ASCII:  .......4....

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: -0.0343628
[INFO] [1758355095.264630416] [rclcpp]: Actuator 5 position: -0.034363
[INFO] [1758355095.264670095] [rclcpp]: Reading position of actuator 6 after waiting...
readCurrentPosition called for actuator 0x6
[DEBUG] Sending command (addr=0x06, cmd=0x06): 
Hex dump: EE 06 06 00 00 ED 
ASCII:  ......

Sent 6 bytes
Received 12 bytes
First byte: 0xffffffee
Command byte: 0x6
[DEBUG] Raw response data: 
Hex dump: EE 06 06 00 04 27 00 18 00 00 90 ED 
ASCII:  .....'......

Response size: 12 bytes
Response[0]: 0xee
Response[2]: 0x6
Response[last]: 0xed
Current position: 39.0004
[INFO] [1758355095.265514554] [rclcpp]: Actuator 6 position: 39.000366
[INFO] [1758355095.265546629] [rclcpp]: Disabling actuator 1...
[DEBUG] Sending command (addr=0x01, cmd=0x2A): 
Hex dump: EE 01 2A 00 01 00 BF 40 ED 
ASCII:  ..*....@.

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 01 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355095.270437085] [rclcpp]: Disabling actuator 2...
[DEBUG] Sending command (addr=0x02, cmd=0x2A): 
Hex dump: EE 02 2A 00 01 00 BF 40 ED 
ASCII:  ..*....@.

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 02 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355095.275380933] [rclcpp]: Disabling actuator 3...
[DEBUG] Sending command (addr=0x03, cmd=0x2A): 
Hex dump: EE 03 2A 00 01 00 BF 40 ED 
ASCII:  ..*....@.

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 03 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355095.280129498] [rclcpp]: Disabling actuator 4...
[DEBUG] Sending command (addr=0x04, cmd=0x2A): 
Hex dump: EE 04 2A 00 01 00 BF 40 ED 
ASCII:  ..*....@.

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 04 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355095.285112168] [rclcpp]: Disabling actuator 5...
[DEBUG] Sending command (addr=0x05, cmd=0x2A): 
Hex dump: EE 05 2A 00 01 00 BF 40 ED 
ASCII:  ..*....@.

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 05 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355095.289840390] [rclcpp]: Disabling actuator 6...
[DEBUG] Sending command (addr=0x06, cmd=0x2A): 
Hex dump: EE 06 2A 00 01 00 BF 40 ED 
ASCII:  ..*....@.

Sent 9 bytes
Received 7 bytes
First byte: 0xffffffee
Command byte: 0x2a
[DEBUG] Raw response data: 
Hex dump: EE 06 2A 00 01 01 ED 
ASCII:  ..*....

[INFO] [1758355095.295163624] [rclcpp]: All actuators successfully disabled. Exiting.

