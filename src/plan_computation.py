import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_image_footprint_on_surface, compute_ground_sampling_distance, compute_image_footprint_on_surface_with_gimbal_angle, compute_focal_length_in_mm


def compute_distance_between_images(camera: Camera, dataset_spec: DatasetSpec) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        float: The distance between images in the horizontal direction.
        float: The distance between images in the vertical direction.
    """
    footprint_x, footprint_y = compute_image_footprint_on_surface(camera, dataset_spec.height)

    distance_x = footprint_x * (1 - dataset_spec.overlap)
    distance_y = footprint_y * (1 - dataset_spec.sidelap)

    return np.array([distance_x, distance_y])


def compute_distance_between_images_with_gimbal_angle(camera: Camera, dataset_spec: DatasetSpec) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        float: The distance between images in the horizontal direction.
        float: The distance between images in the vertical direction.
    """
    footprint_x, footprint_y = compute_image_footprint_on_surface_with_gimbal_angle(camera, dataset_spec.height, dataset_spec.gimbal_x_deg, dataset_spec.gimbal_y_deg)

    distance_x = footprint_x * (1 - dataset_spec.overlap)
    distance_y = footprint_y * (1 - dataset_spec.sidelap)

    return np.array([distance_x, distance_y])

def compute_speed_during_photo_capture(camera: Camera, dataset_spec: DatasetSpec, allowed_movement_px: float = 1) -> float:
    """Compute the speed of drone during an active photo capture to prevent more than 1px of motion blur.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        allowed_movement_px (float, optional): The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        float: The speed at which the drone should move during photo capture.
    """
    gsd = compute_ground_sampling_distance(camera, dataset_spec.height)
    return gsd * allowed_movement_px / (dataset_spec.exposure_time_ms / 1000)

def generate_photo_plan_on_grid(camera: Camera, dataset_spec: DatasetSpec) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        List[Waypoint]: scan plan as a list of waypoints.

    """
    distance_x, distance_y = compute_distance_between_images(camera, dataset_spec)

    num_of_images_x = math.ceil(dataset_spec.scan_dimension_x / distance_x)
    num_of_images_y = math.ceil(dataset_spec.scan_dimension_y / distance_y)

    speed = compute_speed_during_photo_capture(camera, dataset_spec)

    footprint_x, footprint_y = compute_image_footprint_on_surface(
        camera, dataset_spec.height
    )
    dx = footprint_x / 2
    dy = footprint_y / 2

    waypoints = []

    for ny in range(num_of_images_y):
        y = ny * distance_y
        surface_coord_y1 = y - dy
        surface_coord_y2 = y + dy

        for nx in range(num_of_images_x):
            if ny % 2 == 0:
                x = nx * distance_x
            else:
                x = (num_of_images_x - nx - 1) * distance_x

            surface_coord_x1 = x - dx
            surface_coord_x2 = x + dx
            waypoints.append(
                Waypoint(
                    x,
                    y,
                    surface_coord_x1,
                    surface_coord_x2,
                    surface_coord_y1,
                    surface_coord_y2,
                    dataset_spec.height,
                    speed
                )
            )

    return waypoints


def generate_photo_plan_on_grid_with_gimbal_angle(
    camera: Camera, dataset_spec: DatasetSpec
) -> T.List[Waypoint]:
    distance_x, distance_y = compute_distance_between_images_with_gimbal_angle(camera, dataset_spec)

    num_of_images_x = math.ceil(dataset_spec.scan_dimension_x / distance_x)
    num_of_images_y = math.ceil(dataset_spec.scan_dimension_y / distance_y)

    speed = compute_speed_during_photo_capture(camera, dataset_spec)

    fx_mm, fy_mm = compute_focal_length_in_mm(camera)

    gimbal_x_rad = np.radians(dataset_spec.gimbal_x_deg)
    gimbal_y_rad = np.radians(dataset_spec.gimbal_y_deg)

    fov_x = 2 * np.arctan((camera.sensor_size_x_mm / 2) / fx_mm)
    fov_y = 2 * np.arctan((camera.sensor_size_y_mm / 2) / fy_mm)

    dx1 = dataset_spec.height * np.tan(gimbal_x_rad - fov_x / 2)
    dx2 = dataset_spec.height * np.tan(gimbal_x_rad + fov_x / 2)
    dy1 = dataset_spec.height * np.tan(gimbal_y_rad - fov_y / 2)
    dy2 = dataset_spec.height * np.tan(gimbal_y_rad + fov_y / 2)

    waypoints = []

    for ny in range(num_of_images_y):
        y = ny * distance_y
        surface_coord_y1 = y + dy1
        surface_coord_y2 = y + dy2

        for nx in range(num_of_images_x):
            if ny % 2 == 0:
                x = nx * distance_x
            else:
                x = (num_of_images_x - nx - 1) * distance_x

            surface_coord_x1 = x + dx1
            surface_coord_x2 = x + dx2
            waypoints.append(
                Waypoint(
                    x,
                    y,
                    surface_coord_x1,
                    surface_coord_x2,
                    surface_coord_y1,
                    surface_coord_y2,
                    dataset_spec.height,
                    speed
                )
            )

    return waypoints


def compute_travel_time_between_waypoints(
    distance_m: float,
    max_speed_m_per_sec: float,
    max_acceleration_m_per_sec2: float,
    capture_speed_m_per_sec: float,
) -> float:
    # assumptions: acceleration and deceleration are at the same rate
    acceleration_time = (
        max_speed_m_per_sec - capture_speed_m_per_sec
    ) / max_acceleration_m_per_sec2
    acceleration_distance = (capture_speed_m_per_sec * acceleration_time) + (
        max_acceleration_m_per_sec2 * (acceleration_time**2) / 2
    )

    if (acceleration_distance * 2) > distance_m:
        # triangular speed profile
        acceleration_time = math.sqrt((distance_m / 2) / max_acceleration_m_per_sec2)
        total_time = 2 * acceleration_time
    else:
        # trapezoidal speed profile
        d_constant = distance_m - (2 * acceleration_distance)
        t_constant = d_constant / max_speed_m_per_sec
        total_time = (2 * acceleration_time) + t_constant

    return total_time


def compute_total_time_for_photos(
    waypoints: T.List[Waypoint],
    max_speed_m_per_sec: float,
    max_acceleration_m_per_sec2: float,
    capture_speed_m_per_sec: float,
    capture_time_ms: float,
) -> float:
    total_time = 0.0

    distance_travelled_while_capturing = capture_speed_m_per_sec * (
        capture_time_ms / 1000
    )

    for i in range(1, len(waypoints)):
        distance = np.sqrt(
            (waypoints[i].x - waypoints[i - 1].x) ** 2
            + (waypoints[i].y - waypoints[i - 1].y) ** 2
        )

        # reduce the distance travelled by the drone while capturing the photo
        distance -= distance_travelled_while_capturing

        travel_time = compute_travel_time_between_waypoints(
            distance,
            max_speed_m_per_sec,
            max_acceleration_m_per_sec2,
            capture_speed_m_per_sec,
        )

        total_time += travel_time + capture_time_ms

    # n-1 capture time is added, add the last capture time
    if len(waypoints) > 0:
        total_time += capture_time_ms

    return total_time
