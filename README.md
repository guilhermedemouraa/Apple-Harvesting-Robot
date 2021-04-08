# Apple-Harvesting-Robot

![final](https://user-images.githubusercontent.com/39603677/113951001-80325380-97c7-11eb-9470-c83c07979b81.gif)


## Project Description
Scaled harvesting robot using Parallax ActivityBot 360o to simulate apple picking. This project was inspired in the 2018 ASABE Student Robotics Challenge held in Detroit, MI.
The challenge consisted in creating robot to go over a simulated apple orchard, locate and classify apples (green, ripe, and overripe), leave green apples behind, harvest ripe apples, remove overripe apples, and provide a summarized report of the operation.

![Orchard](https://user-images.githubusercontent.com/39603677/113948821-a9041a00-97c2-11eb-878f-3bb8de0f2ebc.png)
![class field](https://user-images.githubusercontent.com/39603677/113948820-a86b8380-97c2-11eb-95fd-0fb9d441b27a.png)

Fig. 1. (a) Robot-compatible apple orchard, Yakama Valley, WA. Source: 2018 ASABE Student Robotics Challenge. (b) Simulated apple orchard

•	The key steps for the success of this project included developing code for remote operation through a smartphone, path following, PID control, and data collection, classification (logistic regression), and analysis.

• Path following was assisted by a laser range finder, a gyroscope, and ultrasonic sensors. In addition, the robot was provided with PID control.
• The identification of the "apples" (colored ping-pong balls) was assisted by a dichromatic optical reflectance sensor. Data classification was performed through logistic regression.
• Apple picking was performed through a mechanic arm controlled by a servo motor.

![Robot design](https://user-images.githubusercontent.com/39603677/113948823-a99cb080-97c2-11eb-8ce8-cb6cc00f340d.JPG)

Fig. 2. Mechanic arm for apple picking

• The last part of the project consisted in creating a summary report of the apple harvesting operation, which was presented to the user in a local webpage, developed in HTLM.

![Picture1](https://user-images.githubusercontent.com/39603677/113950191-99d29b80-97c5-11eb-9d65-b4089882aa7c.png)

Fig. 3 Report summary

