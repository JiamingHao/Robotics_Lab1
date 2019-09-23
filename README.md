# W4733 Robotics Lab 1

* **Usage**:
>Run the following four commands in order and each in a separate terminal:
>1. roscore  
>2. roslaunch rbx1_bringup fake_turtlebot.launch
>3. rosrun rviz rviz -d \`rospack find rbx1_nav\`/sim.rviz
>4. rosrun rbx1_nav timed_out_and_back.py

* **Methods**:
> - **translations(self, dis_str):** Take the translation distance as a string and perform the translations, handles both positive and negative translaions.
> - **rotations(self, ang_str):** Take the rotation angle in degree and perform the rotation, positive angle for counterclockwise rotation, negative angle for clockwise rotation.
> - **shutdown(self):** Stop the robot when shutting down the node.

* **Helper Function**:
> - **is_number(s):** Tell whether the input string is numeric, return True for yes, otherwise return False

* **Others**:
> The main program can only be terminated by typing in 'Q' when prompted to  choose from 'T, R or Q' operations

* **Video Link**

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/HVGM2JbwUqk/0.jpg)](http://www.youtube.com/watch?v=HVGM2JbwUqk)
https://youtu.be/HVGM2JbwUqk
