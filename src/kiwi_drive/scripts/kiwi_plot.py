import numpy as np
from utils import rotate_edges
import plotly.io as pio
import plotly.graph_objs as go

class PlotlyPlot:
    def __init__(self) -> None:
    #    self.robot = robot
       self.frames = ""
    def plot(list_of_robots):
            robot_trajectories = {}
            robot_edges = {}
            for index, robot in enumerate(list_of_robots):
                robot_trajectories[f"robot_{robot.robot_id}"] = [robot.Xw, robot.Yw, robot.Tp]
                robot_edges[f"robot_{robot.robot_id}"] = [rotate_edges(robot.Xw, robot.Yw, robot.Tp)]

            x1, y1, x2, y2, x3, y3 = rotate_edges(x.copy(), y.copy(), theta.copy())

            # add this in the functionality later
            xm = np.min(x) - 1.5
            xM = np.max(x) + 1.5

            ym = np.min(y) - 1.5
            yM = np.max(y) + 1.5

            N = np.size(x)
            z = np.zeros(N)


            frames=[go.Frame(
                        data=[go.Scatter3d(
                            x=[x[k]],
                            y=[y[k]],
                            z =[z[k]],
                            mode="markers",
                            marker=dict(color="red", size=5,)),
                            go.Scatter3d(
                            x=[x1[k]],
                            y=[y1[k]],
                            z = [z[k]],
                            mode="markers",
                            marker=dict(color="blue", size=5,)),
                            go.Scatter3d(
                            x=[x2[k]],
                            y=[y2[k]],
                            z = [z[k]],
                            mode="markers",
                            marker=dict(color="yellow", size=5,)),
                            go.Scatter3d(
                            x=[x3[k]],
                            y=[y3[k]],
                            z = [z[k]],
                            mode="markers",
                            marker=dict(color="orange", size=5,))
                            ],
                            name=str(k),
                            layout={"title": "Kiwi Kinematics",
                                    "scene": {
                                        "xaxis": {"title": "X", "range": [-2, 2], "color": "white",
                                                "linewidth": 3},
                                        "yaxis": {"title": "Y", "range": [-2, 2], "color": "white",
                                                "linewidth": 3},
                                        "zaxis": {"title": "Z", "range": [-0.5, 0.5], "color": "white",
                                                "linewidth": 3},
                                        "aspectratio": {"x": 1, "y": 1, "z": 1},
                                        "bgcolor":"rgba(0, 0, 0, 1)"}
                                    })

                        for k in range(N)]

            # Create figure
            fig = go.Figure(
                data=[go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                    go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                    go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                    go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                    go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                    go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                    go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                    go.Scatter3d(x=x, y=y, z = z,
                                mode="lines",
                                line=dict(width=10, color="blue")),
                                ],

                    layout={"title": "Kiwi Kinematics",
                            "updatemenus": [{"type": "buttons",
                                            "buttons": [{
                                                "label": "Play",
                                                "method": "animate",
                                                "args": [None]}]
                                            }
                                            ],
                            "scene": {
                                "xaxis": {"title": "X", "range": [-2, 2]},
                                "yaxis": {"title": "Y", "range": [-2, 2]},
                                "zaxis": {"title": "Z", "range": [-0.5, 0.5]},
                                "aspectratio": {"x": 1, "y": 1, "z": 1},
                                "bgcolor":"rgba(0, 0, 0, 1)"}
                            },

                frames=frames
            )

            fig.show()