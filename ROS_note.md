# ROS Catkin Workspace Notes

## Workspace & Build

* **Build all packages**

  ```bash
  catkin_make
  ```

* **Build a specific package**

  ```bash
  catkin_make -DCATKIN_WHITELIST_PACKAGES="my_pkg"
  ```

* **Build multiple specific packages**

  ```bash
  catkin_make -DCATKIN_WHITELIST_PACKAGES="pkg1;pkg2"
  ```

* **Reset whitelist (build all packages again)**

  ```bash
  catkin_make -DCATKIN_WHITELIST_PACKAGES=""
  ```

* **Important:** After every build, source the workspace to update environment variables:

  ```bash
  source devel/setup.bash
  ```

* **Tip:** If build issues occur, clean and rebuild:

  ```bash
  rm -rf build devel
  catkin_make
  ```

---

## Custom Messages

1. Create a `msg/` folder inside your package.

2. Add a `.msg` file, e.g. `pwmVal.msg`:

   ```msg
   int32 left
   int32 right
   ```

3. Modify **CMakeLists.txt**:

   ```cmake
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     std_msgs
     message_generation
   )

   add_message_files(
     FILES
     pwmVal.msg
   )

   generate_messages(
     DEPENDENCIES
     std_msgs
   )

   catkin_package(
     CATKIN_DEPENDS message_runtime std_msgs
   )
   ```

4. Modify **package.xml** (don’t forget this step):

   ```xml
   <build_depend>message_generation</build_depend>
   <exec_depend>message_runtime</exec_depend>
   ```

---

## Launch Files

* Used to start multiple ROS nodes with a single command.

* Place launch files inside:

  ```
  catkin_ws/src/your_package/launch/
  ```

* Example (`your_launch_file.launch`):

  ```xml
  <launch>
    <node name="publisher_node" pkg="ros_task2" type="publisher_node" output="screen"/>
    <node name="subscriber_node" pkg="ros_task2" type="subscriber_node" output="screen"/>
  </launch>
  ```

* **Tip:** Use `<arg>` for configurable parameters and `<rosparam>` to load YAML parameter files.

---

## ros\_lib for Arduino

If your package defines custom messages and you are using Arduino with `rosserial`, update `ros_lib` after building:

```bash
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries
```

---

## Additional Tips

* **Install dependencies automatically:**

  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```

* **Auto-source your workspace:**
  Add this line to `~/.bashrc` to avoid sourcing manually each time:

  ```bash
  source ~/catkin_ws/devel/setup.bash
  ```

* **Multiple terminals:** Tools like `tmux` or `terminator` help manage multiple ROS terminals.

---

