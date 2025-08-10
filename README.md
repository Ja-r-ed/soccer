# Project Report: Omnidirectional Soccer Wheelchair
### Aiden Blanchfield, year 10
### Jared Jin, year 10
### Glen Waverley Secondary College
## 1. Introduction
There has been a very large push in recent years to get people with all levels of ability into
sports, particularly those with disabilities. This is why wheelchair basketball exists: for people
with a lack of leg movement. However, soccer is an immensely popular sport with relatively
little representation in the disability sports field. As such, the aim of this project was to create
a soccer wheelchair robot that can be used by people with arm and leg disabilities, such as
cerebral palsy. This type of wheelchair already exists, called Power Soccer; however, it
works through a ‘tank’ drive and requires the user to spin on the spot to kick the ball and
pivot on the spot to change direction. Our wheelchair robot will instead use an
omnidirectional drive design, which will allow the wheelchair robot to rotate while moving,
better mimicking a real soccer player. We will also have a ‘dribbler’ integrated into our
design, to dribble and then kick the ball rather than spinning the wheelchair to hit the ball.
## 2. Aims
Our aims are the following:
To create a functional scale model soccer wheelchair robot design for users with limited limb
movement, using motors and a controller to move around. It must have an omnidirectional
drive base that allows for holonomic movement, unlike the Power Soccer robot design, and a
dribbler/kicker mechanism that allows the ball to be ‘dribbled’ and then kicked like real
soccer.
## 3. Operating Instructions
Our wheelchair robot must have easy-to-use, intuitive controls. There must be a main switch
to turn the whole wheelchair on and off, and then a control panel for the user. As different
users of the wheelchair will have different abilities in their motor control, a simple interface
should be designed to accommodate different users' capabilities. The following controls
should be implemented on the robot:
- Rotation, joystick
- Translation, joystick
- Dribble, switch
- Kick, button
## 4. Design Brief
The entire wheelchair robot was first created in CAD (computer-aided design), as most parts
are going to be created using 3D printing, requiring a computer model. All non-custom parts
were put in CAD before being purchased, which enabled a higher degree of precision due to
recognising which parts would not work through constraints such as time, size, etc. The CAD
model went through several revisions, from basic layouts, to first sketches and then the final
version that was 3D printed in real life.
The omniwheels of the robot are constructed out of four 3D-printed layers, with metal dowel
pins in between to allow the outer wheels to rotate. Each of these omniwheels are driven by
a 970 RPM electric gearmotor from ServoCity, which was chosen due to having high speed
and torque for its small size, while not being expensive. Each motor pair is powered by a
Cytron 16V 3A motor driver, which provides plenty of power to the motors while not breaking
the bank nor size limit. All components in the robot are driven by an SMC True Spec 4S
battery (14.8V), which goes through a 5V buck converter to power the ESP32s, the brains of
the robot. Also within the robot is an Adafruit 9-DoF IMU (Inertial Measurement Unit), which
handles the robot's position through its accelerometer and gyroscope. The dribbler, featured
on the front of the robot, is powered by a small DC motor from Jaycar. This is run by a
L298N motor driver, also controlled by the ESP32s.
Software for this robot was written in C++ with the ESP-IDF development framework for the
ESP32. The code for this robot can be found in the appendix. One ESP32 was used as a
sensor/calculations board while the other ESP32 was used to drive motors (movement,
dribbling and kicking). The “sensors” ESP32 is the brain, while the “motors” ESP32 is used
to “respond” to the brain's commands.
## 7. Discussion
Overall, this exploration of an omnidirectional wheelchair robot for disabled users found that
this is a viable solution for said users. This addresses a real societal issue wherein disabled
people face difficulty doing things that everyone else can do, such as playing soccer. It is
efforts such as these that impactfully improve the involvement of people with disabilities in
everyday life. An omnidirectional wheelchair robot such as this one could be scaled up in
size and used as an improvement to the Power Soccer wheelchair robot, using a 4
omniwheel holonomic drivebase instead of the movement-limiting “tank” drive, a motorised
dribbler instead of a metal bar and a solenoid kicker instead of the same metal bar.
By the end of the assembly process, many changes were made to improve the accuracy and
functionality of the completed wheelchair robot. These changes include swapping the ESP32
microcontroller for an Arduino Mega, as both ESP32s did not behave in repeatable or
understandable ways. This reflects how reproducibility in results is more important than
short-term successes, as although the ESP32s have many better features than the Arduino
Mega, their failure in creating reproducible and repeatable results meant that they were
inadequate for the task. Another change is the addition of elastic bands onto our dribbler to
help it grip the ball, as the lack of friction caused by the plastic on plastic contact of the 3D
printed dribbler and ping-pong ball caused the dribbler to slip when attempting to dribble the
ball. Elastic bands greatly increased the grip of the dribbler and better solutions (such as 3D
printing in flexible plastic) will be looked into in further iterations.
In future revisions of this project (possibly the one brought to judging day), we aim to
improve our wheelchair robot by implementing computer vision-aided aiming and more
advanced dribbling and kicking mechanisms to better assist the user. Many challenges were
faced throughout the course of this project, in particular time and money. Funding was
provided by the school, albeit three weeks from the due date, and the entire robot was made
over the course of five school days, with errors in the 3D printing and the coding process not
making this any less difficult.
## 8. References
robocup-junior 2020, GitHub - robocup-junior/awesome-rcj-soccer: A curated list of resources relevant to RoboCupJunior Soccer, GitHub, viewed 27 July 2025, <https://github.com/robocup-junior/awesome-rcj-soccer/> 

Small Size League | RoboCup Soccer – Information for the administration of the RoboCup Soccer Small Size League n.d., RoboCup Small Size League, viewed 27 July 2025, <https://ssl.robocup.org/>

Open-Source Kicker Circuit - Platform for creating and sharing projects - OSHWLab, Oshwlab.com, viewed 27 July 2025, <https://oshwlab.com/afgasgasdg/open-source-kicker-circuit> 

RoboCupJunior, viewed 27 July 2025, <https://junior.robocup.org/> Austermann, N, Davis Imai, K, Alexeevich Korennykh, D & Mark Ogata, S n.d., Lightweight Soccer Robot with Dribbler, Kicker, and Vector Projections: Orion 2021, viewed 27 July 2025, <https://robocup-junior.github.io/soccer-2021/pdfs/TDPs/LWL_Orion.pdf>. 

Imai, K 2022, RoboCupJunior Open, Keiji Imai, viewed 27 July 2025, <https://kogappa.com/projects/rcj_open/> 

Arduino n.d., Arduino Docs | Arduino Documentation, <http://docs.arduino.cc>

ESP-IDF Programming Guide - ESP32 - — ESP-IDF Programming Guide v5.2.1 documentation n.d., <http://docs.espressif.com> 

## 9. Acknowledgements
Mr. Lynch-Wells, for organising all our meetings for building this wheelchair robot, and for ordering the parts and putting up with us while we tweaked and tweaked and tweaked. 

Mr. Newbold and Ms Silva, for organising the STS side of this and helping us write this report and nudging us in the right direction for this report. 

Mr. McIntosh and Mr. Martinek, for letting us 3D print and test our robot at all hours of the day. 

Aldrich Liem, for giving us constant advice and lending us equipment to help make this happen. 

## 10. Appendix
### Logbook:
30/3/25 - Started CAD model
- Created first models of premade parts
- Started work on omniwheels CAD
- Began research on different components, such as motor controllers
- Started basic codebase
- Created github project
- Started on basic drivebase code

18/4/25 - Continued working on CAD model
- Found viable motor options
- Created motor CADs
- Found possible sensors, such as the IMU

30/5/25 - Continued working on CAD model
- Found all parts required
- Began basic layout drafts
- Decided on battery

31/5/25 - Finished the omniwheels
- Refined motor controller, motor and microcontroller choices to current
- Colour coded parts
- Created another draft layout with new components

1/6/25 - Started first revision of actual bottom parts
- Created motor mounts
- Created top-bottom spacers
- Implemented standoffs for all PCBs
- Finished basic drivebase code
- Tuned to fit cad parameters
- Started camera testing with Pixy2
- Started computer vision code

13/6/25 - Created the first draft of the top half
- Mounted the ESP32s and battery
- Still need to mount dribbler motor and controller
- First revision of dribbler drum
- Ordered parts!

20/7/25 - Finished CAD!
- Mounted dribbler drum
- Mounted dribbler motor and motor driver
- Added cable routing holes
- Pocketed base to remove weight
- Added pulleys to dribbler motor
- Needs to be printed to begin assembly

22/7/25 - Printed most parts
- Began bottom half assembly
- Began top half assembly
- M3 were too small, so drilled out larger
- Finished printing parts
- Began wiring most parts together
- Finished assembly of wheels and dribbler subsystems
- Finished basic motor code
- Finished robot
- Basic testing
- Finished code
- Made modifications (see discussion):
- Moved from ESP32 to an Arduino Mega because we accidently killed both of our ESP32s…
- Because of no ESP32 (and thus no wifi/bluetooth functionality), we used a IR (infrared) remote
- Used much simpler code without the complex math because it was more reliable 
### Finished CAD:
https://cad.onshape.com/documents/fc0a5e085ca7f56b4458a98d/w/5e97d10eac742d85865ba744/e/c55f60db607fb82f3faf3afa?renderMode=0&uiState=6885d45944cc0165960b1b70 
### Finished code:
https://github.com/Ja-r-ed/SoccerSTS
