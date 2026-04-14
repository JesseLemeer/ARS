import math
import numpy as np
def kalman_filter(x,y,theta,sigma_sq_x,sigma_sq_y, sigma_sq_theta, sigma_sq_Rx,sigma_sq_Ry, sigma_sq_Rtheta, v,omega,dt):
    #prediction
    A = np.identity(3)
    R = np.array([(sigma_sq_Rx,0,0),(0,sigma_sq_Ry),(0,0,sigma_sq_Rtheta)])
    u = np.array([[v],[omega]])
    B = np.array([(dt*math.cos(theta),0),(dt*math.sin(theta),0),(0,dt)])
    mu_old = ([[x],[y],[theta]])
    mu_new = np.dot(A,mu_old) + np.dot(B,u)
    print(mu_new)
    x_new = mu_new[0,0]
    y_new = mu_new[1,0]
    theta_new = mu_new[2,0]
    sigma_old= np.array([(sigma_sq_x,0,0),(0,sigma_sq_y),(0,0,sigma_sq_theta)])
    sigma_new = A @ sigma_old @ A.T + R
    
    #correction

kalman_filter(1,2,3,4,5,1)