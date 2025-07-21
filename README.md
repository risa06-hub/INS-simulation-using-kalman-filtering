# INS-simulation-using-kalman-filtering
A personal simulation project that demonstrates how an Inertial Navigation System (INS) estimates the position and velocity of a moving object using noisy accelerometer and gyroscope data.
This project implements Kalman Filters to estimate the motion of objects under two different dynamics:

1. 🚗 **2D Circular Motion**: Simulates a body moving in a circular path using noisy accelerometer and gyroscope data, with GPS-like position/velocity corrections.
2. 🛰️ **Satellite Trajectory Simulation**: Models the motion of a satellite in a simplified orbit (extendable to full orbital mechanics), using simulated inertial and GPS data.

Both simulations estimate the full state (position and velocity) using **Kalman filtering** and demonstrate basic **sensor fusion**.

---

