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
### THEORY
---
#### Basic Kalman Filter Explanation for INS

A Kalman Filter is used to estimate the state of a system (like position, velocity, orientation) even when the sensor measurements are noisy.

**Kalman Filter Steps**
 
-Define the state vector which will store the information about the state of the object like position and velocity
   
   for example:
     x=[ x
         y
         vx
         vy
           ]
   
-**prediction step**
   we use physics to predict the next state:
   
   ``` xk∣k−1​=A⋅xk−1∣k−1​+B⋅uk```
   
   where
   -A: state transition matrix
   
   -B: control matrix
   
   -uk: acceleration input from accelerometer
   
-**update step**

  use the measurement from the GPS to correct the predicted values
  
  ```xk∣k​=xk∣k−1​+K⋅(zk​−H⋅xk∣k−1​)```
  where 
  -zk: measurement
  
  -H: measurement matrix
  
  -K: kalman gain

-**update error covaiance matrix**

--- 

#### P matrix
P is the state covariance matrix.

-It encodes your uncertainty about the system’s state vector x.

-It’s always a square matrix with dimensions equal to the number of state variables.

 -``` P= [  [Var(x) Cov(y,x) Cov(vx,x) Cov(vy,x)] ,
            [​Cov(x,y) Var(y) Cov(vx,y) Cov(vy,y)] , 
            [​Cov(x,vx) Cov(y,vx) Var(vx) Cov(vy,vx)​],
            [Cov(x,vy) Cov(y,vy) Cov(vx,vy) Var(vy)] ​]```
 
 -Each element tells the filter how uncertain it is about relationships between different parts of the state.

In Kalman filtering prediction:

-```P= A @ P @ A.T```

-``` Pk∣k−1​=A⋅Pk−1∣k−1​⋅A⊤+Q```

---

#### How does the state transition matrix formed
If:

  The object moves with constant velocity, and

   Time step is ΔtΔt,

Then:

      xk+1 = xk + vx⋅Δt
      
      yk+1= yk + vy⋅Δt 
      
      vxk+1 = vxk 
      
      vyk+1 = vyk

​​​**covert to matrix**

    xk+1 ​ = F⋅xk​

    F=[1 0 Δt 0 
    
       0 1 0 Δt 
       
       0 0 1 0 
       
       0 0 0 1 ]​

---

#### Kalman Gain

The Kalman gain tells is a weighting factor that determines how much to trust the measurement vs the current prediction.

Kalman Gain Formula:

```K = P @ H.T @ S-1```

where S=(H @ P @H.T + R) 

it indicated the total uncertanity in the predicted measurement

 ```H @ P @ H.T``` 
 
 – Transforms state uncertainty into measurement space

If S is large → we don’t trust the measurement much → small correction

If S is small → we trust the measurement more → bigger correction

**what is S?**

S is the innovation covariance matrix.It quantifies the total uncertainty in the measurement residual and is used to compute Kalman gain(K)

Kalman Update Equation:

The Kalman gain is used in the update step like this:

```x_updated = x_predicted + K⋅(z − H @ x_predicted)```

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
```





## Author

[Risa Mahajan](https://github.com/risa06-hub)













