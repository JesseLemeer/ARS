import math
import numpy as np
import pygame

def draw_covariance_ellipse(
    surface,
    kf_est_x: float,
    kf_est_y: float,
    sigma_xx: float,
    sigma_yy: float,
    sigma_xy: float,
    world_to_screen, 
    screen_width:  int,
    screen_height: int,
    color=(255, 127, 0),
    n_std: int   = 2,
    n_points: int = 64,
    thickness: int = 2,
):

    # Build 2×2 covariance sub-matrix
    cov = np.array([[sigma_xx, sigma_xy],
                    [sigma_xy, sigma_yy]], dtype=float)

    # Eigen-decomposition
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    eigenvalues = np.maximum(eigenvalues, 0.0)

    # Largest eigenvalue represents major axis
    idx   = np.argsort(eigenvalues)[::-1]
    evals = eigenvalues[idx]
    evecs = eigenvectors[:, idx]

    # Semi-axis lengths in world units
    a = n_std * math.sqrt(evals[0])   # major
    b = n_std * math.sqrt(evals[1])   # minor

    # Rotation: angle of the major eigenvector
    angle = math.atan2(evecs[1, 0], evecs[0, 0])

    # Generate ellipse vertices in world space
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    screen_pts = []

    for i in range(n_points):
        t = 2.0 * math.pi * i / n_points

        # Point on axis-aligned ellipse
        ex = a * math.cos(t)
        ey = b * math.sin(t)

        # Rotate into world frame
        wx = kf_est_x + ex * cos_a - ey * sin_a
        wy = kf_est_y + ex * sin_a + ey * cos_a

        # Project to screen
        sx, sy = world_to_screen(wx, wy, screen_width, screen_height)
        screen_pts.append((sx, sy))

    # Draw
    if len(screen_pts) >= 3:
        pygame.draw.polygon(surface, color, screen_pts, thickness)

def get_ellipse_axes(sigma_xx, sigma_yy, sigma_xy, n_std=2):
    """
    Return (width, height, angle_deg) suitable for matplotlib's Ellipse patch.
    Used in experiments.py for offline plot generation.
    """
    cov = np.array([[sigma_xx, sigma_xy],
                    [sigma_xy, sigma_yy]], dtype=float)
    evals, evecs = np.linalg.eigh(cov)
    evals = np.maximum(evals, 0.0)
    idx   = np.argsort(evals)[::-1]
    evals, evecs = evals[idx], evecs[:, idx]

    w = 2 * n_std * math.sqrt(evals[0])
    h = 2 * n_std * math.sqrt(evals[1])
    angle_deg = math.degrees(math.atan2(evecs[1, 0], evecs[0, 0]))
    return w, h, angle_deg