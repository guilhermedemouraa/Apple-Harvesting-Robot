# Apple-Harvesting-Robot

![final](https://user-images.githubusercontent.com/39603677/113951001-80325380-97c7-11eb-9470-c83c07979b81.gif)

## Disclaimer:
While every attempt has been made to accurately create a mobile robot simulation, the work has not been verified independently and some typographical and/or technical errors may be present due to the type of assumptions made. In any case, the authors (Guilherme De Moura Araujo and Kaiyan Li) hope that the present files will be a helpful guide and assist in making further improvements as necessary.

The project you will find in this repository is directly taken from course assignments with the instructor’s consent.

## Project Description
Scaled harvesting robot using [Parallax ActivityBot 360o](https://www.parallax.com/product/activitybot-360-robot-kit/) to simulate apple picking. This project was inspired in the [2018 ASABE Student Robotics Challenge](https://abe.ufl.edu/precag/pdf/2018RoboticsRules.pdf) held in Detroit, MI.
The challenge consisted in creating robot to go over a simulated apple orchard, locate and classify apples (green, ripe, and overripe), leave green apples behind, harvest ripe apples, remove overripe apples, and provide a summarized report of the operation.

![Orchard](https://user-images.githubusercontent.com/39603677/113948821-a9041a00-97c2-11eb-878f-3bb8de0f2ebc.png)
![class field](https://user-images.githubusercontent.com/39603677/113948820-a86b8380-97c2-11eb-95fd-0fb9d441b27a.png)

Fig. 1. (a) Robot-compatible apple orchard, Yakama Valley, WA. Source: 2018 ASABE Student Robotics Challenge. (b) Simulated apple orchard

-	The key steps for the success of this project included developing code for remote operation through a smartphone, navigation/path following, and data collection, classification (logistic regression), and analysis.

### Navigation
- Navigation/path following was assisted by a laser range finder, a [gyroscope](https://www.parallax.com/product/gyroscope-module-3-axis-l3g4200d/), [ultrasonic sensors](https://www.parallax.com/product/ping-ultrasonic-distance-sensor/) (ping), and PID control. A closed-loop feedback control algorithm was developed for all sensors.

### Apple picking
- The identification of the "apples" (colored ping-pong balls) was assisted by a dichromatic optical reflectance sensor. 
- We ran the robot over the field to collected sensor data that was later annotaded and used to create a classification model through logistic regression. In total, there were 5 classes: (1) black (noisy data from the background of the simulated orchard field); (2) white (walls of the simulated orchard field); (3) green (immature apples that should be left untouched); (4) pink (mature apples that should be harvested); and (5) red (overripe apples that should be knocked out)
- Apple picking was performed through a mechanic arm controlled by a servo motor.

![Robot design](https://user-images.githubusercontent.com/39603677/113948823-a99cb080-97c2-11eb-8ce8-cb6cc00f340d.JPG)

Fig. 2. Mechanic arm for apple picking. Source: Slaughter, D.C., EBS289K: Robotics, smart machines, and IoT. 2018.

### Live Wi-Fi dashboard with operation summary report
- The last part of the project consisted in creating a live Wi-Fi dashboard (hosted by the robot) to present a summary of the operation. The dashboard was developed using HTLM, CSS, and JavaScript.

![Picture1](https://user-images.githubusercontent.com/39603677/113950191-99d29b80-97c5-11eb-9d65-b4089882aa7c.png)

Fig. 3 Report summary

## Files
- appleHarvesting_main.c: Masterfile for the project. Initializes parameters, calls all functions, performs navigation and C.V. algorithms, and sends data via Wi-Fi to update the live dashboard.
- appleHarvesting_main.side: Auxiliary to "appleHarvesting_main.c". This file has some steps to login on an external party authentication provider.
- web_application.html: Source code for our live Wi-Fi dasboard.
