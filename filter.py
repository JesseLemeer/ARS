import math
import numpy as np
from itertools import combinations

#ChatGPT
def _wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

#ChatGPT
def triangulation(measurement_a, measurement_b):
    x1, y1 = measurement_a["hit_point"]
    x2, y2 = measurement_b["hit_point"]
    r1 = measurement_a["distance"]
    r2 = measurement_b["distance"]
    phi1 = math.radians(measurement_a["angle_deg"])
    phi2 = math.radians(measurement_b["angle_deg"])

    dx = x2 - x1
    dy = y2 - y1
    d = math.hypot(dx, dy)

    if d == 0.0:
        return None
    if d > r1 + r2 or d < abs(r1 - r2):
        return None

    a = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
    h_sq = r1 ** 2 - a ** 2
    if h_sq < 0.0:
        if h_sq > -1e-9:
            h_sq = 0.0
        else:
            return None

    h = math.sqrt(h_sq)
    mid_x = x1 + a * dx / d
    mid_y = y1 + a * dy / d

    offset_x = -dy * h / d
    offset_y = dx * h / d
    candidates = [
        (mid_x + offset_x, mid_y + offset_y),
        (mid_x - offset_x, mid_y - offset_y),
    ]

    best_pose = None
    best_error = float("inf")

    for robot_x, robot_y in candidates:
        global_angle_1 = math.atan2(y1 - robot_y, x1 - robot_x)
        global_angle_2 = math.atan2(y2 - robot_y, x2 - robot_x)
        theta_1 = _wrap_angle(global_angle_1 - phi1)
        theta_2 = _wrap_angle(global_angle_2 - phi2)
        error = abs(_wrap_angle(theta_1 - theta_2))

        if error < best_error:
            best_error = error
            best_pose = (robot_x, robot_y, _wrap_angle((theta_1 + theta_2) / 2.0))

    return best_pose

#If we observe several landmarks, we use the average pose estimated over all combinations of landmarks
def get_estimated_pose(measurements):
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
    theta_avg = math.atan2(sum(math.sin(pose[2]) for pose in poses), sum(math.cos(pose[2]) for pose in poses))
    
    #theta_avg is in radians
    print(f"Estimated pose: X = {x_avg:.2f}, Y = {y_avg:.2f}, theta = {math.degrees(theta_avg):.2f} deg")

    return x_avg, y_avg, theta_avg


def kalman_filter(x,y,theta,sigma_sq_x,sigma_sq_y, sigma_sq_theta, sigma_sq_Rx,sigma_sq_Ry, sigma_sq_Rtheta, sigma_sq_Qx,sigma_sq_Qy, sigma_sq_Qtheta, v,omega,dt, measurements):
    #prediction
    A = np.identity(3)
    R = np.array([(sigma_sq_Rx,0,0),(0,sigma_sq_Ry),(0,0,sigma_sq_Rtheta)])
    u = np.array([[v],[omega]])
    B = np.array([(dt*math.cos(theta),0),(dt*math.sin(theta),0),(0,dt)])

    mu_old = ([[x],[y],[theta]])
    mu_new_bar = np.dot(A,mu_old) + np.dot(B,u)

    sigma_old= np.array([(sigma_sq_x,0,0),(0,sigma_sq_y),(0,0,sigma_sq_theta)])
    sigma_new_bar = A @ sigma_old @ A.T + R
    
    #correction

    estimated_pose = get_estimated_pose(measurements)
    if estimated_pose is None:
        return mu_new_bar, sigma_new_bar

    x_bar, y_bar, theta_bar = estimated_pose

    eps_x = np.random.normal(0, 1.0)
    eps_y = np.random.normal(0, 1.0)
    eps_theta = np.random.normal(0, 0.05)

    z = np.array([[x_bar + eps_x],
                [y_bar + eps_y],
                [theta_bar + eps_theta]])

    C = np.eye(3)
    Q = np.array([(sigma_sq_Qx,0,0),(0,sigma_sq_Qy),(0,0,sigma_sq_Qtheta)])

    K = sigma_new_bar @ C.T @ np.linalg.inv(C@sigma_new_bar@C.T + Q)

    mu_new = mu_new_bar + K@(z-C@mu_new_bar)
    sigma_new = (np.eye(3)-K@C)@sigma_new_bar

    return mu_new, sigma_new
