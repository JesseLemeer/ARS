import math
import numpy as np
from itertools import combinations

#Normalizes any radian angle into the range [-pi, pi]
def normalize_angle(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def triangulation(measurement_a, measurement_b):
    x1, y1 = measurement_a["hit_point"]
    x2, y2 = measurement_b["hit_point"]
    r1 = measurement_a["distance"]
    r2 = measurement_b["distance"]
    phi1 = math.radians(measurement_a["angle_deg"])
    phi2 = math.radians(measurement_b["angle_deg"])

    #Distance between the two landmarks
    dx = x2 - x1
    dy = y2 - y1
    #Euclidean distance between them
    d = math.sqrt(dx**2 + dy**2)

    #Check for same landmark, can happen when the robot is very close to a landmark
    #If both measurements hit the same landmark we cannot use this
    #Radius of a landmark is 5, so distance between is max 10
    #We assume there is no two landmarks closer than 10 to each other
    if d <= 10.0:
        return None
    #Check for no intersection or one circle inside the other
    if d > r1 + r2 or d < abs(r1 - r2):
        return None
    
    #Distance from x1 to the perpendicular crossing point
    projection = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
    
    #Height from the crossing point to the intersection points
    h_sq = r1 ** 2 - projection ** 2
    
    #Non-negative check and floating point guard
    if h_sq < 0:
        if h_sq > -1e-6:  # Allow for small numerical errors
            h_sq = 0
        else:   
            return None
    
    h = math.sqrt(h_sq)
   
    cross_x = x1 + projection * dx / d
    cross_y = y1 + projection * dy / d

    offset_x = -dy * h / d
    offset_y = dx * h / d
    
    #Now we have two candidate positions 
    candidates = [
        (cross_x + offset_x, cross_y + offset_y),
        (cross_x - offset_x, cross_y - offset_y)
    ]

    best_pose = None
    best_error = math.inf

    #Keep the candidate with the smallest angle error
    for candidate_x, candidate_y in candidates:
        global_angle_1 = math.atan2(y1 - candidate_y, x1 - candidate_x)
        global_angle_2 = math.atan2(y2 - candidate_y, x2 - candidate_x)
        
        theta_1 = normalize_angle(global_angle_1 - phi1)
        theta_2 = normalize_angle(global_angle_2 - phi2)
        error = abs(normalize_angle(theta_1 - theta_2))

        if error < best_error:
            best_error = error
            best_pose = (candidate_x, candidate_y, normalize_angle((theta_1 + theta_2) / 2.0))

    return best_pose

#If we observe several landmarks, we use the average pose estimated over all combinations of landmarks
def get_estimated_pose(measurements, debug=True):
    if len(measurements) < 2:
        return None

    poses = []
    for measurement_a, measurement_b in combinations(measurements, 2):
        estimated_pose = triangulation(measurement_a, measurement_b)
        if estimated_pose is not None:
            poses.append(estimated_pose)

    if not poses:
        return None

    x_avg = sum(pose[0] for pose in poses) / len(poses)
    y_avg = sum(pose[1] for pose in poses) / len(poses)
    theta_avg = normalize_angle(
        math.atan2(
            sum(math.sin(pose[2]) for pose in poses),
            sum(math.cos(pose[2]) for pose in poses),
        )
    )

    if debug:
        print(f"Estimated pose: X = {x_avg:.2f}, Y = {y_avg:.2f}, theta = {theta_avg:.3f} rad")

    return x_avg, y_avg, theta_avg


def kalman_filter(x,y,theta,sigma_sq_x,sigma_sq_y, sigma_sq_theta, sigma_sq_Rx,sigma_sq_Ry, sigma_sq_Rtheta, sigma_sq_Qx,sigma_sq_Qy, sigma_sq_Qtheta, v,omega,dt, measurements):
    #prediction
    A = np.identity(3)
    R = np.array([(sigma_sq_Rx,0,0),(0,sigma_sq_Ry,0),(0,0,sigma_sq_Rtheta)])
    u = np.array([[v],[omega]])
    B = np.array([(dt*math.cos(theta),0),(dt*math.sin(theta),0),(0,dt)])

    mu_old = np.array([[x],[y],[normalize_angle(theta)]])
    mu_new_bar = np.dot(A,mu_old) + np.dot(B,u)
    mu_new_bar[2, 0] = normalize_angle(mu_new_bar[2, 0])

    sigma_old= np.array([(sigma_sq_x,0,0),(0,sigma_sq_y,0),(0,0,sigma_sq_theta)])
    sigma_new_bar = A @ sigma_old @ A.T + R
    
    #correction

    #Return prediction if no pose can be estimated from measurements
    estimated_pose = get_estimated_pose(measurements, debug=False)
    if estimated_pose is None:
        return mu_new_bar, sigma_new_bar

    x_bar, y_bar, theta_bar = estimated_pose

    C = np.eye(3)
    Q = np.array([(sigma_sq_Qx,0,0),(0,sigma_sq_Qy,0),(0,0,sigma_sq_Qtheta)])

    eps_x = np.random.normal(0, math.sqrt(sigma_sq_Qx))
    eps_y = np.random.normal(0, math.sqrt(sigma_sq_Qy))
    eps_theta = np.random.normal(0, math.sqrt(sigma_sq_Qtheta))

    z = np.array([[x_bar + eps_x],
                  [y_bar + eps_y],
                  [normalize_angle(theta_bar + eps_theta)]])

    # z = np.array([[x_bar],
    #               [y_bar],
    #               [normalize_angle(theta_bar)]])

    K = sigma_new_bar @ C.T @ np.linalg.inv(C@sigma_new_bar@C.T + Q)

    innovation = z - C @ mu_new_bar
    innovation[2, 0] = normalize_angle(innovation[2, 0])

    mu_new = mu_new_bar + K @ innovation
    mu_new[2, 0] = normalize_angle(mu_new[2, 0])
    sigma_new = (np.eye(3)-K@C)@sigma_new_bar

    return mu_new, sigma_new
