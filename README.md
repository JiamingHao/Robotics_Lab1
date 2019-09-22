# Robotics_Lab1

* **Usage**:  
>1. `roscore`  
>2. `roslaunch rbx1_bringup fake_turtlebot.launch`
>3. `rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz`
>4. `rosrun rbx1_nav timed_out_and_back.py` 

>Run the above commands in order and in separate terminals.

* **Instance Methods**:
> - **translations(self, dis_str):** Take the translation distance as a string and perform the translations, handles both positive and negative translaions.
> - **rotations(self, ang_str):** Take the rotation angle in degree and perform the rotation, positive angle for counterclockwise rotation, negative angle for clockwise rotation.
> - **shutdown(self):** Stop the robot when shutting down the node.
* **Helper Function**:
> - **is_number(s):** Tell whether the input string is numeric, return True for yes, otherwise return False

* **Video Link**
