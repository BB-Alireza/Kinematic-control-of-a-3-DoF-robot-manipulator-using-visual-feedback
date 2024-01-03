In the '[PERSIS](https://sites.google.com/view/ansari-a/persis)-Eye Synchronization' phase of my MS project, it is crucial to develop a comprehensive understanding of the entire process, addressing various challenges such as lighting and filtering in visual feedback, as well as selecting the appropriate control approach. To expedite my knowledge acquisition in this area, I opted to design and fabricate a robotic manipulator for a relatively simple tracking scenario. 

Another crucial factor was 'Camera Selection.' After evaluating budget-friendly options available in the market, I chose the Logitech C505E. This decision was influenced by its capability to adjust focus and modify its focus point using a simple lens holder. Additionally, it is compatible with the eye-tracking system I am working on. Once the camera was selected, I defined the scenario. Initially, the center of the object grasped by the robot end effector is determined. Subsequently, the control objective is established, wherein the robot endeavors to align the center of the grasped object with the center of the image. Finally, I employed a kinematic control method to implement this scenario. Below are my notes on designing the controller. Additionally, I used an Arduino UNO to implement my control algorithm, and the joint actuator I chose is the SG90 servo motor.

