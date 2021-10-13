# Dc-Motor-Control-Pid
Dc motor control with pid algorithm
- 12V dc motor with encoder, l298n motor driver, 12V power supply and arduino was used as hardwarea 
- Pid library was not used.
- DC motor control was realized between 100 and 10000 rpm
- By counting the rising and falling edges of the outputs A and B, 64 pulses are received in one complete revolution of the motor shaft.
- Pid algorithm runs every 50 ms
- You should enter the desired rpm value RPM_d

Encoder A -> pin2 <br>
Encoder B -> pin3 <br>
L298N in1 -> pin7 <br>
L298N in2 -> pin8 <br>
L298N enA -> pin6 <br>
Encoder + -> 5V <br>
Encoder - -> Ground <br>
L298N + -> 12V <br>
L298N - -> Ground <br>
