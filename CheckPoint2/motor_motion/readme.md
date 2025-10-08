
## Checkpoint2 Overview
This checkpoint demonstrates how to control a two-wheeled mobile robot using a Raspberry Pi, Arduino, and the L298N motor driver.  
The Raspberry Pi receives user inputs as PWM values for the left and right motors, publishes these values via **ROS topics**, and the Arduino subscribes to these topics. The Arduino then forwards the PWM signals to the **L298N driver**, which controls the motors with feedback control.

With this setup, the vehicle can perform:
- Forward movement  
- Left turns  
- Right turns  
- Reverse driving
- 3m Straight line test

---

## System Architecture
1. **Raspberry Pi**  
   - Receives user input for left and right motor PWM values.  
   - Publishes PWM commands via `pubPWM`.  

2. **Arduino**  
   - Subscribes to the Raspberry Pi’s ROS topic.  
   - Sends received PWM values to the **L298N motor driver**.  

3. **L298N Motor Driver**  
   - Controls the motors’ direction and speed based on PWM input.  

4. **PID Controller**  
   - Ensures stable motor speed control.  
   - Compensates for differences between the two motors to minimize drift.  

---

