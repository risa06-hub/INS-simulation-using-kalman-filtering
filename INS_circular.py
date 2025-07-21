import numpy as np
import matplotlib.pyplot as plt

omega=0.5
t_circle=2*np.pi/omega
num_rev=2
T=num_rev*t_circle

dt=0.01
N=int(T/dt)
t=np.linspace(0,T,N)

radius=5


#true values
true_theta=omega*t
true_x=radius*np.cos(omega*t)
true_y=radius*np.sin(omega*t)
true_vx=-radius*omega*np.sin(omega*t)
true_vy=radius*omega*np.cos(omega*t)
true_ax=-radius*omega*omega*np.cos(omega*t)
true_ay=-radius*omega*omega*np.sin(omega*t)

#accelerometer (in body frame) simulation
acc_noise=0.1*np.random.rand(N,2)
gyro_noise=0.01*np.random.rand(N)
gyro_z=np.full(N,omega)

acc_body=np.zeros((N,2))
for i in range(N):
    theta=true_theta[i]
    R=np.array([[np.cos(theta),np.sin(theta)],
                [-np.sin(theta),np.cos(theta)]])
    acc_body[i]=R@np.array([true_ax[i],true_ay[i]]) + acc_noise[i]

#kalman filter
x=np.zeros((4,1)) #state vector: [x,y,vx,vy]
P=np.eye(4)*500   #covariance matrix with variance=500(very high). 4X4 matrix

A=np.array([[1,0,dt,0],      #state transition matirx for x,y,vx,vy
            [0,1,0,dt],
            [0,0,1,0],
            [0,0,0,1]])

B=np.array([[0.5*dt*dt,0],    #control matrix for acceleration
            [0,0.5*dt*dt],
            [dt,0],
            [0,dt]])


H=np.eye(4) #measurement matrix 4x4 identity matrix
Q=np.eye(4)*0.01 #process noise
R=np.eye(4)*4

#GPS noise simulation
measured_pos=np.stack([true_x,true_y],axis=1) + 2*np.random.rand(N,2) #position measured with random noise with std=2
measured_vel=np.stack([true_vx,true_vy],axis=1) + 1*np.random.rand(N,2)
measurements=np.hstack((measured_pos,measured_vel))
est_theta=np.zeros(N)
#kalman prediciton

x_est=[]
for i in range(N):
    u=acc_body[i] #measured quantity from the accelerometer
    x= A @ x + B @ u.reshape(2,1)
    P= A @ P @ A.T +  Q

    #updating the measurement
    z= measurements[i].reshape(4,1)
    y= z - H @ x
    S= H @ P @ A.T + R #total uncertanity
    K= P @ H.T @ np.linalg.inv(S) #kalman gain, where np.linalg.inv(S) creates inverse of S matrix

    x= x + K @ y #updated position values
    P= (np.eye(4)-K @ H) @ P
    x_est.append(x.flatten())

    #estimatio of theta
    if i>0:
        est_theta[i]=est_theta[i-1]+gyro_z[i]*dt

x_est=np.array(x_est)

#plotting

plt.figure(figsize=(10,6))
plt.plot(true_x,true_y,label="True path")
plt.plot(x_est[:,0],x_est[:,1],label="kalman estimated path")
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("2D Kalman Filter: Circular Motion with INS")
plt.legend()
plt.axis("equal")
plt.grid()
plt.show()










