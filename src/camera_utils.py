"""Utility functions for the camera model.
"""
import numpy as np

from src.data_model import Camera

def compute_focal_length_in_mm(camera: Camera) -> np.ndarray:
    """Computes the focal length in mm for the given camera

    Args:
        camera (Camera): the camera model.

    Returns:
        np.ndarray: [fx, fy] in mm.
    """
    # Note(Ayush): Solution provided by project leader.
    pixel_to_mm_x = camera.sensor_size_x_mm / camera.image_size_x_px
    pixel_to_mm_y = camera.sensor_size_y_mm / camera.image_size_y_px

    return np.array([camera.fx * pixel_to_mm_x, camera.fy * pixel_to_mm_y])

def project_world_point_to_image(camera: Camera, point: np.ndarray) -> np.ndarray:
    """Project a 3D world point into the image coordinates.

    Args:
        camera (Camera): the camera model
        point (np.ndarray): the 3D world point

    Returns:
        np.ndarray: [u, v] pixel coordinates corresponding to the point.
    """
    # object point
    xo, yo, zo = point

    # image point
    xi = camera.fx * xo / zo
    yi = camera.fy * yo / zo

    # pixel coordinates
    u = xi + camera.cx
    v = yi + camera.cy

    return np.array([u, v])


def compute_image_footprint_on_surface(camera: Camera, distance_from_surface: float) -> np.ndarray:
    """Compute the footprint of the image captured by the camera at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).

    Returns:
        np.ndarray: [footprint_x, footprint_y] in meters.
    """
    footprint_x = distance_from_surface * (camera.image_size_x_px / camera.fx)
    footprint_y = distance_from_surface * (camera.image_size_y_px / camera.fy)

    return np.array([footprint_x, footprint_y])


def compute_image_footprint_on_surface_with_gimbal_angle(camera: Camera, distance_from_surface: float, gimbal_x_deg: float, gimbal_y_deg: float) -> np.ndarray:
    fx_mm, fy_mm = compute_focal_length_in_mm(camera)

    gimbal_x_rad = np.radians(gimbal_x_deg)
    gimbal_y_rad = np.radians(gimbal_y_deg)

    fov_x = 2 * np.arctan((camera.sensor_size_x_mm / 2) / fx_mm)
    fov_y = 2 * np.arctan((camera.sensor_size_y_mm / 2) / fy_mm)

    footprint_x = distance_from_surface * (np.tan(gimbal_x_rad + fov_x / 2) - np.tan(gimbal_x_rad - fov_x / 2))
    footprint_y = distance_from_surface * (np.tan(gimbal_y_rad + fov_y / 2) - np.tan(gimbal_y_rad - fov_y / 2))

    return np.array([footprint_x, footprint_y])


def compute_ground_sampling_distance(camera: Camera, distance_from_surface: float) -> float:
    """Compute the ground sampling distance (GSD) at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
    
    Returns:
        float: the GSD in meters (smaller among x and y directions).
    """
    footprint_x, footprint_y = compute_image_footprint_on_surface(camera, distance_from_surface)

    gsd_x = footprint_x / camera.image_size_x_px
    gsd_y = footprint_y / camera.image_size_y_px

    return min(gsd_x, gsd_y)


def reproject_image_point_to_world(camera: Camera, pixel: np.ndarray, distance_from_surface: float) -> np.ndarray:
    """Reproject a 2D image point back to a 3D world point.

    Args:
        camera (Camera): the camera model.
        pixel (np.ndarray): the 2D pixel location [u, v].
        depth (float): the depth (distance from the camera) of the point.

    Returns:
        np.ndarray: the 3D world point [x, y, z].
    """
    u, v = pixel

    # Convert pixel coordinates to image coordinates
    xi = (u - camera.cx) / camera.fx
    yi = (v - camera.cy) / camera.fy

    # Compute the 3D world coordinates
    x = xi * distance_from_surface
    y = yi * distance_from_surface
    z = distance_from_surface

    return np.array([x, y, z])
