|---SDK2.1.12           Includes the dynamic library files and header files of the released content
     |---Linux
          |---C&C++          DLLs for C&C++
               |---x86_64-Linux-gnu     DLL compiled by this Linux compiler
          |---python3        DLLs for Python
               |---x86_64-Linux-gnu      DLL compiled by this Linux compiler
     |---Window
          |---C&C++          DLLs for C&C++
               |---x64          64-bit compiled DLL
          |---Python3        DLLs for Python
               |---x64          64-bit compiled DLL
     |---example            Some functional examples
|---doc                      Documentation for SDK (Python, C, C++ and C#) and TCP

Note:
Before using the SDK on Windows, please install the Microsoft Visual C++ Redistributable Package.exe. 

V2.1.11--------->V2.1.14 Update Notes:
Fix:
1. Redefine 10004 port mechanism to solve the problem of SDK crash caused by 10004.
2. Redefine motion block mechanism to solve the problem that motion block can't work.
3. Solve the problem of inpos not being allowed to be in place.
Add.
1. add set_user_var() and get_user_var() interfaces for setting and getting system variables.
2. add get_motion_status interface: get motion related status information.
3. Add embedded S part of the adaptation interface
- add zero_end_sensor(), get_tool_drive_state(), get_tool_drive_frame(), set_tool_drive_frame()
- add get_fusion_drive_sensitivity_level(), set_fusion_drive_sensitivity_level()
- add get_compliant_speed_limit(), set_compliant_speed_limit()
- add get_torque_ref_point(), set_torque_ref_point()
- add get_end_sensor_sensitivity_threshold(), set_end_sensor_sensitivity_threshold()
Header file:
1. Update all header file comments to English.
2. Add copyright information and version information in front of the header file. 


V2.1.7--------->V2.1.11 update description:
1. fix and improve 10004 communication mechanism
2. add the interface to get the path of SDK logs.
3. fix the problem that set_ft_ctrl_frame interface can't be used.
4. add linear motion interface with attitude velocity and attitude acceleration
5. add arc motion interface with arc center point motion mode

V2.1.4/V2.1.5 to V2.1.7 Updates:
Bugs fixed:
1. Fixed the issue where calling Python get_robot_status would raise an exception.
2. Fixed the issue of not correctly obtaining support for realTorque on certain robots.
3. Fixed the get_robot_status crash issue in C#.
4. Fixed the issue in C# where supporting torque sensors caused crashes due to garbled characters.
5. Fixed the issue where send_tio_rs_command could not send values exceeding 0x7f.
6. Fixed the issue of incorrect unit conversion for acceleration in radians in circular_move and circular_move_extend.
7. Optimized occasional issues of obtaining a status of 0 from RobotStatus.
8. Fixed the issue of get_robot_status sometimes not showing changes.
9. Fixed the program crash issue when dealing with an excessive number of trajectory files.

Feature added:
1. Added interfaces for setting/getting the robot installing angle.
2. Introduced certain interfaces to address the issue of incomplete string transmission when using the C# SDK on specific robots.