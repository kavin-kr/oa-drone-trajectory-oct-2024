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
    image_size_x_px: float  # Number of pixels in the image along the x axis
    image_size_y_px: float  # Number of pixels in the image along the y axis


@dataclass
class DatasetSpec:
    """
    Data model for specifications of an image dataset.
    """
    overlap: float              # the ratio (in 0 to 1) of scene shared between two consecutive images
    sidelap: float              # the ratio (in 0 to 1) of scene shared between two images in adjacent rows
    height: float               # the height of the scan above the ground (in meters)
    scan_dimension_x: float     # the horizontal size of the rectangle to be scanned
    scan_dimension_y: float     # the vertical size of the rectangle to be scanned
    exposure_time_ms : float    # the exposure time for each image (in milliseconds)

    gimbal_x_deg: float = 0     # the tilt angle of the gimbal along the x axis (in degrees)
    gimbal_y_deg: float = 0     # the tilt angle of the gimbal along the y axis (in degrees)


@dataclass
class Waypoint:
    """
    Waypoints are positions where the drone should fly to and capture a photo.
    """
    x: float # x coordinate of the waypoint
    y: float # y coordinate of the waypoint

    # the surface area which will be captured in the image from this waypoint
    # we can recalculate this based on the camera model and the dataset spec,
    # but storing it here will make it easier to visualize the waypoints
    surface_coord_x1: float
    surface_coord_x2: float
    surface_coord_y1: float
    surface_coord_y2: float

    # review: height is inferred from the dataset spec, should we include it here too?
    # z: float

    # review: waypoints are generated based on the gimbal angles from the dataset spec, should we include them here too?
    # gimbal_x_deg: float
    # gimbal_y_deg: float