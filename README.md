# Adaptive Headlight with Real-Time Data Logging and Cloud Integration

## Project Overview
With advancements in intelligent automotive systems, ensuring optimal road safety and visibility is crucial. This project presents an **Adaptive Headlight System** using STM32 microcontrollers to dynamically adjust the vehicle's headlights based on real-time environmental factors. The system also integrates cloud-based data logging for remote monitoring and predictive maintenance.

## Features
- **Real-time Adaptive Headlights:** Automatically adjusts beam intensity and direction based on traffic and road conditions.
- **Dual-Controller Architecture:** Utilizes a Functional Control Unit (FCU) and a Telematics Control Unit (TCU) for system operation.
- **CAN Bus Communication:** Seamless data transfer between FCU and TCU using the **Controller Area Network (CAN)** protocol.
- **Cloud Integration:** Data transmission via **MQTT protocol** for remote access and analytics.
- **Predictive Maintenance:** Logs sensor data for future analysis and safety optimizations.

## System Architecture
### Functional Control Unit (FCU)
- **STM32 Microcontroller:** Manages real-time headlight control.
- **Sensors:**
  - **Light Sensors:** Detect ambient lighting conditions.
  - **Accelerometers & Gyroscopes:** Monitor vehicle movement.
- **Headlight Actuators:** Adjust intensity and direction dynamically.

### Telematics Control Unit (TCU)
- **STM32 Microcontroller:** Handles data processing and cloud communication.
- **CAN Bus Interface:** Facilitates data exchange with the FCU.
- **Communication Module:** Uses **MQTT over Wi-Fi/LTE** for cloud data transfer.
- **Data Storage Module:** Logs real-time sensor information.

## Communication and Data Flow
1. **Sensor Data Acquisition:** FCU collects environmental and vehicle movement data.
2. **Headlight Adjustment:** Based on real-time data, the FCU dynamically modifies the headlight settings.
3. **Data Transmission to TCU:** The FCU sends operational data to the TCU via CAN bus.
4. **Cloud Upload:** The TCU processes and transmits logged data to the cloud using MQTT.
5. **Remote Monitoring:** Cloud dashboard allows real-time visualization and analytics.

## Hardware Components
- **Microcontrollers:** STM32F407-based FCU and TCU.
- **Sensors:** Light Dependent Resistor (LDR), accelerometer, and gyroscope.
- **Communication Modules:** CAN transceiver, Wi-Fi/LTE module.
- **Actuators:** Servo motors for headlight angle control.

## Software Implementation
- **Embedded C Programming:** STM32 firmware development.
- **CAN Bus Protocol:** Enables real-time data transfer.
- **MQTT Communication:** Secure cloud data logging and analytics.
- **Data Logging & Analysis:** Historical tracking for predictive maintenance.


## Future Enhancements
- **AI-based Predictive Analytics:** Optimize headlight behavior using AI-driven insights.
- **Mobile/Web App Development:** Enable user control and diagnostics.
- **Enhanced Sensor Suite:** Integrate additional road condition sensors.
- **Optimized Servo Control:** Improve response time and accuracy of headlight adjustments.

## Contributors
- **L. Omkar**  
- **P. Aniket**  
- **P. Vinay**  
- **SK. Faizulla**  
- **T. Vamsi**  

## Acknowledgments
- Guided by **Mr. Arafat Khan**
- Developed at **Centre for Development of Advanced Computing (C-DAC), ACTS Pune**

## References
- [CAN Bus Communication Protocol](https://www.sae.org/)
- [STM32 Cloud Solutions](https://www.st.com/)
- [Ansys AVxcelerate Headlamp Simulation](https://www.ansys.com/)

