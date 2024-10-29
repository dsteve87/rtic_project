import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
 
# define the robot parameters
# wheel radius 
r=15
# distance between the wheel centers
s=4*r
 
# time vector
timeVector=np.linspace(0,100,10000)
 
# here you define the robot control variables
# left wheel angular velocity
dphiLA=2*np.ones(timeVector.shape)
# right wheel angular velocity
dphiRA=1.4*np.ones(timeVector.shape)
 
# this function defines a system of differential equations 
# defining the forward kinematics 
# x[0]=x, x[1]=y, x[2]=theta
def diffModel(x,t,timePoints,sC,rC,dphiLArray,dphiRarray):
    #value of dphiL at the current time t 
    dphiLt=np.interp(t,timePoints, dphiLArray)
    #value of dphiR at the current time t 
    dphiRt=np.interp(t,timePoints, dphiRarray)
     
    # \dot{x}
    dxdt=0.5*rC*dphiLt*np.cos(x[2])+0.5*rC*dphiRt*np.cos(x[2])
    # \dot{x}
    dydt=0.5*rC*dphiLt*np.sin(x[2])+0.5*rC*dphiRt*np.sin(x[2])
    # \dot{\theta}
    dthetadt=-(rC/sC)*dphiLt+(rC/sC)*dphiRt
     
    # right-side of the state equation
    dxdt=[dxdt,dydt,dthetadt]
    return dxdt
     
 
 
# define the initial values for simulation 
# x,y,theta
initialState=np.array([500,500,0])
# solve the forward kinematics problem
solutionArray=odeint(diffModel,initialState,timeVector,
                     args=(timeVector,s,r,dphiLA,dphiRA))
 
# save the simulation data
np.save('simulationData.npy', solutionArray)
 
# plot the results 
plt.plot(timeVector, solutionArray[:,0],'b',label='x')
plt.plot(timeVector, solutionArray[:,1],'r',label='y')
plt.plot(timeVector, solutionArray[:,2],'m',label='theta')
plt.xlabel('time')
plt.ylabel('x,y,theta')
plt.legend()
plt.savefig('simulationResult.png',dpi=600)
plt.show()