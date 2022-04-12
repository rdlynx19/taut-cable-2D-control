import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from math import pi,sin,cos,tan


def euler_dynamics(x,v,a,t):
    v=v+a*t
    x=x+v*t
    return v,x


#simulation time 
tspan=np.arange(0,8,0.01)
t=800
# Drone params
J= 0.015
mass = 2
gravity = 9.81
rho = 0.1


# Desired control objectives
# alpha_des = -pi/8
# theta_des = 5*pi/6
# r_des = 10


# Current Angles
# alpha = 0
# theta = 0


# Derivative of control objectives
r_dot=0
r_ddot = 0
alpha_dot=0
alpha_ddot = 0
theta_dot=0
theta_ddot = 0

delta_t=0.01

# PD controllers for alpha-(from equation uAlpha)
kP_alpha = 6
kD_alpha = 5

#equation 24
kP_theta = 1
kD_theta = 3
kD_r= 6
kP_r= 6

#arrays
r_arr = [] 
r_d_arr = []
theta_arr= []
theta_d_arr = []
alpha_arr = []
alpha_d_arr = []

initial_state = np.array([1,pi/15,pi/8 ,0,0,0]) 
r=initial_state[0]
theta = initial_state[1]
alpha = initial_state[2]

# Solved_state= In each it will be updated to reach the final desired state , in varad's code 
                                                                        # in simulate function it can be seen.

des_state = np.array([1*sin(9*pi/10),1*cos(9*pi/10),-pi/32,0,0,0]) # r,theta,alpha,r_dot,theta_dot,alpha_dot
alpha_des = 9*pi/10
theta_des=-pi/32
r_des=5
solvedState=[]
solvedState_d = [r_dot,theta_dot,alpha_dot]

# just for animation
y=[]
x=[]

y_dot=[]
x_dot=[]





for i in range(t):
    
    
    u3= -( kD_r*r_dot+ kP_r*(r-r_des) )/rho 
    T_bar = mass*gravity*(cos(alpha_des)*tan(alpha_des+theta_des)-sin(alpha_des))
    uT = T_bar + mass*gravity*sin(alpha) + mass*rho*u3  
    uAlpha = mass*(2*r_dot*alpha_dot + gravity*cos(alpha)) - mass*r*(kP_alpha*(alpha - alpha_des) + kD_alpha*(alpha_dot))
    
    
    theta_C = math.atan2(uT ,uAlpha) - alpha

    u1 = uT/(sin(alpha + theta_C))

    
    theta_error= theta-theta_C
    u2 = -1*J*(kP_theta*theta_error + kD_theta*theta_dot)

    
    #euler_dynamicsx
    r_ddot = rho*u3
    #alpha_ddot = (-(2*r_dot*alpha_dot + gravity*cos(alpha))/r) + (((cos(theta + alpha))/mass*r)*u1)
    alpha_ddot = - (2*r_dot*alpha_dot + gravity*cos(alpha))/r + ((cos(theta_C + alpha))/(mass*r))*u1
    theta_ddot = u2/J

#   Euler approach for solving euler_dynamics

    #  for calculating r
    r_dot,r = euler_dynamics(r,r_dot,r_ddot,delta_t)
    r_d_arr.append(r_dot)     
    r_arr.append(r)

    #  for calculating theta
    theta_dot,theta=euler_dynamics(theta,theta_dot,theta_ddot,delta_t)
    theta_d_arr.append(theta_dot)
    theta_arr.append(theta)
    
    
    # for calculating alpha
    alpha_dot,alpha=euler_dynamics(alpha,alpha_dot,alpha_ddot,delta_t)
    alpha_d_arr.append(alpha_dot)
    alpha_arr.append(alpha)


    # converting to cartesian coordinate
    y.append(r*sin(alpha))
    x.append(r*cos(alpha))
    y_dot.append(r*cos(alpha)*alpha_dot+ sin(alpha)*r_dot)
    x_dot.append(-r*sin(alpha)*alpha_dot + cos(alpha)*r_dot)
print(x[0],y[0])

    
    

    
    
solvedState = np.array([y,x,theta_arr,y_dot,x_dot,theta_d_arr])
solvedState = np.transpose(solvedState)
# solvedState = np.resize(solvedState,(1000,6))
print(solvedState.shape)


    # row = np.array[r,theta,alpha]
    # # solvedState.append([r,theta,alpha])
    # solvedState = np.append(solvedState,[row],axis = 0)


plot1=plt.figure(1)
plt.plot(tspan,theta_arr)
# plt.plot()
plt.xlabel('Time')
plt.ylabel('theta')
# plt.show() 

plot2=plt.figure(2)
plt.plot(tspan,r_arr)
# plt.plot()
plt.xlabel('Time')
plt.ylabel('r')
# plt.show() 

plot3=plt.figure(3)
plt.plot(tspan,alpha_arr)
# plt.plot()
plt.xlabel('Time')
plt.ylabel('alpha')
# plt.show() 

plot4=plt.figure(4)
plt.plot(x,y)
# plt.plot()
plt.xlabel('x')
plt.ylabel('y')
plt.show() 