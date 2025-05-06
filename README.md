# **Linear Inverted Pendulum**

**Author**: Dechatorn Devaphalin

## **1. Introduction**

The inverted pendulum system is inherently unstable and has long been utilized as a benchmark problem in control systems engineering. This project aims to design and construct a one-degree-of-freedom (1-DOF) linear inverted pendulum system. The focus is on developing a physical model equipped with a dynamic mass pendulum and implementing multiple control strategies for system stabilization and evaluation.

---

## **2. Objectives**

The primary objectives of this project are:

* To construct a 1-DOF linear inverted pendulum system.
* To implement and evaluate control algorithms including:

  * Proportional-Integral-Derivative (PID) Control
  * Linear Quadratic Regulator (LQR) Control
  * Kalman Filter-based Estimation
* To allow for variable pendulum mass in order to observe and analyze changes in system dynamics.

---

## **3. System Components**

### 3.1 Mechanical Components

* **Pendulum Rod and Mass**: Configurable with variable weights.
* **Cart and Track Mechanism**: Mounted on aluminum profiles for linear motion.
* **Base Frame**: Constructed using V-slot aluminum extrusions for structural support.
* **Mounts**: Custom-designed mounts for encoder, limit switches, and stepper motor.

### 3.2 Electrical Components

* **Stepper Motor (NEMA 23)**: Drives the cart horizontally.
* **Stepper Driver**: Converts control pulses into motor movement.
* **Rotary Encoder (600 pulses/rev)**: Measures position and speed.
* **Limit Switches**: Prevent over-travel by detecting mechanical limits.
* **Power Supply**: 24V DC regulated power for all components.
* **Microcontroller**: Raspberry Pi Pico used for control implementation.

![Wiring Diagram](https://github.com/user-attachments/assets/5fcde406-794c-494b-8254-54376ac1ca3b)

---

## **4. Software and Libraries**

### 4.1 MicroPython Libraries

* `machine`: For GPIO, PWM, and hardware interface.
* `utime`: For time-based functions such as delays.
* `_thread`: For basic multithreading to simulate real-time operations.

### 4.2 Python Scientific Libraries

* **NumPy**: Used for all matrix and numerical operations.
* **SciPy**: Provides signal processing, matrix operations, and system modeling tools.
* **FilterPy**: For implementing Kalman filters (standard, extended, and unscented).
* **lqr\_control**: Custom library for modeling linear systems and LQR design.

---

## **5. System Modeling**

The system was mathematically modeled through force analysis, leading to derivation of both the physical and electrical equations of motion. These were used to formulate:

* **Transfer Functions**
* **State-Space Representations**

This modeling provided the foundation for controller design and simulation.

---

## **6. System Identification**

To accurately define system parameters, experimental data was collected using a variety of excitation profiles:

* Constant accelerations (1 m/s², 2 m/s², 3 m/s²)
* Sinusoidal inputs (`sin(t/200)`, `cos(t/200)`)

Using MATLAB's optimization tools, including genetic algorithms, parameters such as cart mass (M), pendulum mass (m), moment of inertia (I), and viscous damping (b) were estimated.

| Parameter | Description                       | Unit  | Measured | Estimated |
| --------- | --------------------------------- | ----- | -------- | --------- |
| m         | Mass of the pendulum rod          | kg    | 0.1011   | 0.1025    |
| M         | Mass of the cart                  | kg    | 0.5351   | 0.5471    |
| l         | Pendulum length                   | m     | 0.255    | 0.255     |
| I         | Moment of inertia of pendulum rod | kg·m² | —        | 0.0025    |
| b         | Viscous damping at pivot          | N·m/s | —        | 0.0007    |
| g         | Gravitational constant            | m/s²  | 9.81     | 9.81      |

---

## **7. Control System Design**

### 7.1 PID Controller

A classical PID controller was designed using the root locus method. Simulation responses were validated before hardware implementation. The controller was realized in MicroPython on the Raspberry Pi Pico.

### 7.2 LQR Controller

The system was represented in state-space form, and an LQR controller was designed to minimize a quadratic cost function. Simulation results showed strong stability and optimal control behavior.

### 7.3 Kalman Filter (LQG Control)

A Kalman Filter was designed for state estimation in the presence of noise. The LQR controller was combined with the Kalman Filter to form an LQG controller, which is being prepared for final implementation.

---

## **8. Mechanical Design and Fabrication**

### 8.1 Previous Designs

* **PVC Frame**: Lightweight and cost-effective, but introduced friction and instability.
* **Pneumatic System**: Provided high responsiveness but was expensive and complex.

### 8.2 Final Design

* **Frame**: V-slot aluminum profile for precision and modularity.
* **Cart and Bearings**: Precision bearings and shafts for smooth motion.
* **Encoder and Limit Switch Mounts**: Designed with generative design techniques and 3D-printed using ABS plastic to reduce mass and stiffness.

![Linear Inverted Pendulum Design](https://github.com/user-attachments/assets/3ffbc9cb-f1e5-452e-aff9-dff1cc838d44)

---

## **9. Simulation and Validation**

* **Swing-Up Simulation**: Verified using MATLAB and Python to achieve pendulum stabilization from rest.
* **ANSYS Motion Analysis**: Conducted for mechanical validation of design stresses, displacements, and motion behavior.

---

## **10. Implementation Challenges and Future Work**

* **Vibration and Balancing**: The system experienced excessive vibration during motion, which affected stability.
* **Aesthetic and Hardware Layout**: Future improvements will include better cable management and cleaner physical layout.
* **Kalman Filter Implementation**: Final implementation and tuning of Q and R matrices in MicroPython is planned.

---
