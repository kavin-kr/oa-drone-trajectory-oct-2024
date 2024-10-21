"""Data models for the camera and user specification."""
from dataclasses import dataclass

@dataclass
class Camera:
    """
    Data model for a simple pinhole camera.
    
    References: 
    - https://github.com/colmap/colmap/blob/3f75f71310fdec803ab06be84a16cee5032d8e0d/src/colmap/sensor/models.h#L220
    - https://en.wikipedia.org/wiki/Pinhole_camera_model
    """

    fx: float               # focal length along x axis (in pixels)
    fy: float               # focal length along y axis (in pixels)
    cx: float               # optical center of the image along the x axis (in pixels)
    cy: float               # optical center of the image along the y axis (in pixels)
    sensor_size_x_mm: float # Size of the sensor along the x axis (in mm) [single pixel size * number of pixels in X dimension]
    sensor_size_y_mm: float # Size of the sensor along the y axis (in mm) [single pixel size * number of pixels in Y dimension]
    image_size_x: float     # Number of pixels in the image along the x axis
    image_size_y: float     # Number of pixels in the image along the y axis


@dataclass
class DatasetSpec:
    """
    Data model for specifications of an image dataset.
    """
    pass


@dataclass
class Waypoint:
    """
    Waypoints are positions where the drone should fly to and capture a photo.
    """
    pass