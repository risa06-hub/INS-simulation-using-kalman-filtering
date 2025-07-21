# INS-simulation-using-kalman-filtering
A personal simulation project that demonstrates how an Inertial Navigation System (INS) estimates the position and velocity of a moving object using noisy accelerometer and gyroscope data.


This project implements Kalman Filters to estimate the motion of objects under two different dynamics:

1. 🚗 **2D Circular Motion**: Simulates a body moving in a circular path using noisy accelerometer and gyroscope data, with GPS-like position/velocity corrections.
2. 🛰️ **Satellite Trajectory Simulation**: Models the motion of a satellite in a simplified orbit (extendable to full orbital mechanics), using simulated inertial and GPS data.

Both simulations estimate the full state (position and velocity) using **Kalman filtering** and demonstrate basic **sensor fusion**.

---

## 📌 1. Circular Motion Simulation

### Description

- Simulates **true circular motion** of an object
- Simulates noisy:
  - **Gyroscope** (z-axis angular velocity)
  - **Accelerometer** (in body frame)
  - **GPS-like measurements** of position and velocity
- Implements a **Kalman Filter** for estimating state:  
  \[ x, y, v_x, v_y \]
- Visualizes the **true path vs estimated path**

---

###THEORY

---

### 🔧 How It Works
 1.**simulate Circular Motion**
     -Define radius and angular velocity using basic calculations
     -compute the true values for the position and velocity
     
 2.**Generate Sensor Data**
     -Add noise to accelerometer and gyroscope
     -Rotate acceleration from the world frame to body frame using transformation matrix
     -Add noise to the GPS measurements of position and velocity
    
  3.**Kalman FIlter Setup**
  -state vector:     x= [x,y,vx,vy] 
  -covariance matrix: P, initialised the variance very high,ie 500, because the uncertanity in the position of the object is high
     -matrices:
       -`A` = state transition
       -`B` = control input (acceleration)
       -`H` = measurement matrix (identity)
       -`Q` = process noise (noise from the model, eg:wind,drag etc)
       -`R` = measurement noise (noise from the GPS measurement)
       
  4.**Kalman Filter Loop**
  -prediction step using dynamica and measured acceleration
      
  -update the state vector and covariance matrix using the noise by determining the kalman gain.
      
  -store the estimated state at each time step
    
  5.**Plot the Results**
    -plot the true and kalman estimated path in 2D

---
### Dependencies

- Python 3.x
- NumPy
- Matplotlib
Install with:

```bash
pip install numpy matplotlib
```

### To Run:

```bash
python circular_motion_kalman.py

