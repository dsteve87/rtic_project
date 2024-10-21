import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

r=15
s=4*r

timeVector = np.linspace(0,100,10000)

dphiLA = 2*np.ones(timeVector.shape)
dphiRA = 1.4*np.ones(timeVector.shape)

def diffModel(x,t,timePoints,sC,rC,dphiLArray,dphiRarray):

    dphiLt = np.interp(t,timePoints, dphiLArray)
    dphiRt = np.interp(t,timePoints, dphiRarray)
   

    dxdt = 0.5*rC*dphiLt*np.cos(x[2]) + 0.5*rC*dphiRt*np.cos(x[2])
    dydt = 0.5*rC*dphiLt*np.sin(x[2]) + 0.5*rC*dphiRt*np.sin(x[2])
    dthetadt = -(rC/sC)*dphiLt+(rC/sC)*dphiRt

    dxdt = [dxdt,dydt,dthetadt]
    return dxdt

initialState = np.array([500, 500, 0])
solutionArray= odeint(diffModel, initialState, timeVector, 
                        args=(timeVector,s,r,dphiLA,dphiRA))

np.save('simulationData.npy', solutionArray)

plt.plot(timeVector, solutionArray[:,0],'b',label='x')
plt.plot(timeVector, solutionArray[:,1],'r',label='y')
plt.plot(timeVector, solutionArray[:,2],'m',label='theta')
plt.xlabel('time')
plt.ylabel('x,y,theta')
plt.legend()
plt.savefig('simulationResult.png',dpi=600)
plt.show()

