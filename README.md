# ur_rtde-thread

##### What the project is?

dual arm ur5 install the plate synchronously.

##### Dependence : c++20 、 ur_rtde-1.48 、 ubuntu

##### How to use it ? 

you can follow the ur_rtde compile rule. 

**1.Set dual_arm_synchronously.cpp in /ur_rtdexxx/example/cpp/dual_arm_synchronously.cpp** 

**2.and then  change the following content in  /ur_rtdexxx/Cmakelists.txt**
**line 275-278:**   add the following content behind

```cmake
if(${EXAMPLES})
```

which is:

```cmake
add_executable(dual_arm_synchronously examples/cpp/dual_arm_synchronously.cpp)
target_include_directories(dual_arm_synchronously PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(dual_arm_synchronously PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
```

*line 339:*change the content:

```cmake
set_target_properties(servoj_example forcemode_example speedj_example movej_path_with_blend_example io_example move_async_example move_path_async_example robotiq_gripper_example move_until_contact_example record_data_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
```

as:

```cmake
set_target_properties(dual_arm_synchronously servoj_example forcemode_example speedj_example movej_path_with_blend_example io_example move_async_example move_path_async_example robotiq_gripper_example move_until_contact_example record_data_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
```

**line 349:**add the following content behind:

```camke
 if(${EXAMPLES})
```

which is:

```cmake
add_executable(ur_rtde::dual_arm_synchronously ALIAS dual_arm_synchronously)  
```

**3.and then  add the following content in the end of   /ur_rtdexxx/example/cpp/Cmakelists.txt **

```cmake
add_executable(dual_arm_synchronously dual_arm_synchronously.cpp)
target_link_libraries(dual_arm_synchronously PUBLIC dual_arm_synchronously::rtde)
```

###### 4.create a build directory in /ur_rtdexxx/ , and then open a terminal  to execute the following code:

```shell
cmake ..
make 
```

###### 5.execute the binary file: switch to /ur_rtdexxx/bin/ and  open a terminal  to execute the following code:

```shell
./dual_arm_synchronously
```

###### Result:

https://www.youtube.com/watch?v=0AjZLuIQz50
