import math
import numpy as np
from itertools import combinations

#Normalizes any radian angle into the range [-pi, pi]
def normalize_angle(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def triangulation(measurement_a, measurement_b):
    xa, ya = measurement_a["landmark_center"]
    xb, yb = measurement_b["landmark_center"]
    ra = measurement_a["distance"]
    rb = measurement_b["distance"]
    phia = measurement_a["bearing_rad"]
    phib = measurement_b["bearing_rad"]

    #Distance between the two landmarks
    dx = xb - xa
    dy = yb - ya
    #Euclidean distance between them
    d = math.sqrt(dx**2 + dy**2)

    #Check for same landmarks
    if d == 0:
        return None
    #Check for no intersection or one circle inside the other
    if d > ra + rb or d < abs(ra - rb):
        return None
    
    #Distance from hit point a to radical axis  (line through intersection points)
    projection = (ra ** 2 - rb ** 2 + d ** 2) / (2 * d)
    
    #Intersection of line through hit point a and b with radical axis
    cross_x = xa + projection * dx / d
    cross_y = ya + projection * dy / d
    
    #Height from the crossing point on radical axis to the intersection points
    h_sq = ra ** 2 - projection ** 2
    
    #Non-negative check and floating point guard
    if h_sq < 0:
        if h_sq > -1e-6:
            h_sq = 0
        else:   
            return None
    
    h = math.sqrt(h_sq)

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
        global_angle_a = math.atan2(ya - candidate_y, xa - candidate_x)
        global_angle_b = math.atan2(yb - candidate_y, xb - candidate_x)

        theta_a = normalize_angle(global_angle_a - phia)
        theta_b = normalize_angle(global_angle_b - phib)
        error = abs(normalize_angle(theta_a - theta_b))

        if error < best_error:
            best_error = error
            best_pose = (candidate_x, candidate_y, normalize_angle((theta_a + theta_b) / 2.0))

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

    K = sigma_new_bar @ C.T @ np.linalg.inv(C@sigma_new_bar@C.T + Q)

    innovation = z - C @ mu_new_bar
    innovation[2, 0] = normalize_angle(innovation[2, 0])

    mu_new = mu_new_bar + K @ innovation
    mu_new[2, 0] = normalize_angle(mu_new[2, 0])
    sigma_new = (np.eye(3)-K@C)@sigma_new_bar

    return mu_new, sigma_new

def ekf_filter(x, y, theta,sigma_mat, sigma_R, sigma_Q, v, omega, dt, measurements):

    th = normalize_angle(theta)
 
    mu_bar = np.array([
        [x + v * dt * math.cos(th)],
        [y + v * dt * math.sin(th)],
        [normalize_angle(th + omega * dt)],
    ])
 
    G = np.array([
        [1.0, 0.0, -v * dt * math.sin(th)],
        [0.0, 1.0,  v * dt * math.cos(th)],
        [0.0, 0.0,  1.0],
    ])
 
    sigma_bar = G @ sigma_mat @ G.T + sigma_R

    for reading in measurements:
        lx, ly = reading["landmark_center"]
 
        dx = lx - mu_bar[0, 0]
        dy = ly - mu_bar[1, 0]
        q  = dx**2 + dy**2
 
        if q < 1e-9:
            continue
 
        sqrt_q = math.sqrt(q)
 
        z_hat = np.array([
            [sqrt_q],
            [normalize_angle(math.atan2(dy, dx) - mu_bar[2, 0])],
        ])
 
        z = np.array([
            [reading["distance"]],
            [normalize_angle(reading["bearing_rad"])],
        ])
 
        H = np.array([
            [-dx / sqrt_q, -dy / sqrt_q,  0.0],
            [ dy / q,      -dx / q,      -1.0],
        ])
 
        S = H @ sigma_bar @ H.T + sigma_Q
        K = sigma_bar @ H.T @ np.linalg.inv(S)
 
        innov = z - z_hat
        innov[1,0] = normalize_angle(innov[1, 0])
 
        mu_bar = mu_bar + K @ innov
        mu_bar[2,0] = normalize_angle(mu_bar[2, 0])
        sigma_bar = (np.eye(3) - K @ H) @ sigma_bar
 
    return mu_bar, sigma_bar

def ekf_slam(mu, sigma, sigma_R, sigma_Q, v, omega, dt, measurements, landmark_index):
    n = len(mu)
    th = normalize_angle(mu[2])

    mu_bar = mu.copy()
    mu_bar[0] += v * dt * math.cos(th)
    mu_bar[1] += v * dt * math.sin(th)
    mu_bar[2]  = normalize_angle(th + omega * dt)

    G = np.eye(n)
    G[0, 2] = -v * dt * math.sin(th)
    G[1, 2] =  v * dt * math.cos(th)

    R_full = np.zeros((n, n))
    R_full[:3, :3] = sigma_R

    sigma_bar = G @ sigma @ G.T + R_full

    for reading in measurements:
        lid = reading["landmark_id"]
        r_obs = reading["distance"]
        phi_obs = reading["bearing_rad"]

        if lid not in landmark_index:
            idx = len(landmark_index) * 2
            state_idx = 3 + idx
            landmark_index[lid] = state_idx

            # Expand mu and sigma
            lx_est = mu_bar[0] + r_obs * math.cos(phi_obs + mu_bar[2])
            ly_est = mu_bar[1] + r_obs * math.sin(phi_obs + mu_bar[2])

            new_size = len(mu_bar) + 2
            mu_bar_new = np.zeros(new_size)
            mu_bar_new[:len(mu_bar)] = mu_bar
            mu_bar_new[state_idx]     = lx_est
            mu_bar_new[state_idx + 1] = ly_est
            mu_bar = mu_bar_new

            sigma_new = np.eye(new_size) * 1e6  # large uncertainty for new landmark
            sigma_new[:len(sigma_bar), :len(sigma_bar)] = sigma_bar
            sigma_bar = sigma_new
            n = new_size

        j = landmark_index[lid]
        lx = mu_bar[j]
        ly = mu_bar[j + 1]

        dx = lx - mu_bar[0]
        dy = ly - mu_bar[1]
        q  = dx**2 + dy**2
        if q < 1e-9:
            continue
        sqrt_q = math.sqrt(q)

        # Expected measurement
        z_hat = np.array([
            [sqrt_q],
            [normalize_angle(math.atan2(dy, dx) - mu_bar[2])]
        ])
        z = np.array([[r_obs], [normalize_angle(phi_obs)]])

        H = np.zeros((2, n))
        H[0, 0] = -dx / sqrt_q
        H[0, 1] = -dy / sqrt_q
        H[1, 0] =  dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        H[0, j] =  dx / sqrt_q
        H[0, j+1] = dy / sqrt_q
        H[1, j] = -dy / q
        H[1, j+1] = dx / q

        S = H @ sigma_bar @ H.T + sigma_Q
        K = sigma_bar @ H.T @ np.linalg.inv(S)

        innov = z - z_hat
        innov[1, 0] = normalize_angle(innov[1, 0])

        mu_bar   = mu_bar.reshape(-1, 1)
        mu_bar   = (mu_bar + K @ innov).flatten()
        mu_bar[2] = normalize_angle(mu_bar[2])
        sigma_bar = (np.eye(n) - K @ H) @ sigma_bar

    return mu_bar, sigma_bar, landmark_index