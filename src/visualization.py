"""Utility to visualize photo plans.
"""

import typing as T

import plotly.graph_objects as go

from src.data_model import Waypoint

def plot_photo_plan(computed_plan: T.List[Waypoint], scan_dimension_x: float, scan_dimension_y: float) -> go.Figure:
    """
    Plot the drone flight plan with waypoints and surface coverage using Plotly.

    Args:
        computed_plan (list[Waypoint]): List of waypoints generated for the flight plan.
        scan_dimension_x (float): Total width of the scan area.
        scan_dimension_y (float): Total height of the scan area.

    Returns:
        fig: A Plotly figure object.
    """
    fig = go.Figure()

    # Add scan area rectangle
    fig.add_shape(
        type="rect",
        x0=0,
        y0=0,
        x1=scan_dimension_x,
        y1=scan_dimension_y,
        line=dict(color="black", width=2),
        opacity=0.5,
        name="Scan Area"
    )

    for i, waypoint in enumerate(computed_plan):
        if i > 0:
            # Add line between waypoints
            prev_wp = computed_plan[i - 1]
            fig.add_trace(go.Scatter(
                x=[prev_wp.x, waypoint.x],
                y=[prev_wp.y, waypoint.y],
                mode='lines',
                line=dict(color='blue', width=2),
                showlegend=False
            ))

        # Add waypoint markers
        fig.add_trace(go.Scatter(
            x=[waypoint.x],
            y=[waypoint.y],
            mode='markers+text',
            marker=dict(size=12, color='red', symbol='circle'),
            text=[f"{i+1}"],
            textfont=dict(size=10, color='black'),
            textposition="top center",
            name=f"Waypoint {i+1}"
        ))

    waypoint = computed_plan[0]
    # Add surface coverage area
    fig.add_shape(
        type="rect",
        x0=waypoint.surface_coord_x1,
        y0=waypoint.surface_coord_y1,
        x1=waypoint.surface_coord_x2,
        y1=waypoint.surface_coord_y2,
        line=dict(color="black", width=1),
        fillcolor="#efa093",
        opacity=0.5
    )

    fig.add_annotation(
        x=(waypoint.surface_coord_x1 + waypoint.surface_coord_x2) / 2,
        y=(waypoint.surface_coord_y1 + waypoint.surface_coord_y2) / 2,
        text=f"Footprint of Image 1",
        showarrow=False,
        font=dict(size=10, color="black"),
        align="center",
        bgcolor="white",
        opacity=0.7
    )

    # Get min and max values for x and y coordinates
    x_values = [wp.x for wp in computed_plan] + [wp.surface_coord_x1 for wp in computed_plan] + [wp.surface_coord_x2 for wp in computed_plan] + [scan_dimension_x]
    y_values = [wp.y for wp in computed_plan] + [wp.surface_coord_y1 for wp in computed_plan] + [wp.surface_coord_y2 for wp in computed_plan] + [scan_dimension_y]

    x_min, x_max = min(x_values) - 10, max(x_values) + 10
    y_min, y_max = min(y_values) - 10, max(y_values) + 10

    speed = computed_plan[0].speed_m_per_sec
    altitude = computed_plan[0].z
    number_of_images = len(computed_plan)

    # Update layout with labels and titles
    fig.update_layout(
        title=f"Photo Plan #: {number_of_images}, Speed: {speed:.2f} m/s, Altitude: {altitude} m",
        xaxis_title="X Coordinate (m)",
        yaxis_title="Y Coordinate (m)",
        xaxis=dict(range=[x_min, x_max], showgrid=True),
        yaxis=dict(range=[y_min, y_max], showgrid=True),
        height=800,
        width=1000,
        showlegend=False,
        plot_bgcolor="#e0e7f0"
    )

    return fig
