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
    #Only done when sensor measures at least two landmarks
    #Get coordinates based on sensor readings to landmarks (phi and r)
    x_lm = 
    y_lm =
    theta_lm = 
    
    #Draw noise from normal distribution
    #I don't know which standard deviations...
    eps_x = np.random.normal(0, 1.0)
    eps_y = np.random.normal(0, 1.0)
    eps_theta = np.random.normal(0, 0.05)

    # Measurement vector
    z = np.array([[x_lm + eps_x],
                [y_lm + eps_y],
                [theta_lm + eps_theta]])
    

kalman_filter(1,2,3,4,5,1)