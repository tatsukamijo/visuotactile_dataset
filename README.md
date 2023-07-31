# visuotactile_dataset
A repo for creating a visuotactile dataset based on PyBullet with TACTO vision-based tactile simulator.  

![Screenshot from 2023-07-31 18-32-26](https://github.com/tatsukamijo/visuotactile_dataset/assets/81934527/75f720a4-6891-4de7-a370-995027a0201c)

# Usage
Set your result directory path in `tacto_pose.py` and run it.  
It will save the image from the front vision cam that is displayed at the upper left and the (left-hand) tactile image.

*Note that the gripper is open at first and that it's interactive mode by default, which means you are supposed to control the pose of the grasped object manually by default. 
