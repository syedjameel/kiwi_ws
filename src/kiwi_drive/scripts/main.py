from kiwi_kinematics import KiwiKinematics
import plotly.io as pio
import plotly.graph_objs as go
import numpy as np
from utils import rotate_edges
from PIL import Image
import kaleido
import plotly

if __name__ == "__main__":
    kiwi = KiwiKinematics()

    # # Slide Right
    # v1 = -1.5
    # v2 = 3.0
    # v3 = -1.5
    # Xw, Yw, Tp = kiwi.movement_vel(v1, v2, v3, 5)

    # # Forward
    # v1 = -(np.sqrt(3)/2)*3
    # v2 = 0.0
    # v3 = (np.sqrt(3)/2)*3
    # Xw, Yw, Tp = kiwi.movement_vel(v1, v2, v3, 5)

    # Rotate left
    v1 = 16.0
    v2 = 16.0
    v3 = 16.0
    Xw, Yw, Tp = kiwi.rotate_vel(v1, v2, v3, np.pi/4)

    # Forward
    v1 = -(np.sqrt(3)/2)*3
    v2 = 0.0
    v3 = (np.sqrt(3)/2)*3
    Xw, Yw, Tp = kiwi.movement_vel(v1, v2, v3, 2)

    # Rotate left
    v1 = 16.0
    v2 = 16.0
    v3 = 16.0
    Xw, Yw, Tp = kiwi.rotate_vel(v1, v2, v3, np.pi/4)

    # Forward
    v1 = -(np.sqrt(3)/2)*3
    v2 = 0.0
    v3 = (np.sqrt(3)/2)*3
    Xw, Yw, Tp = kiwi.movement_vel(v1, v2, v3, 2)  

    # Rotate left
    v1 = 16.0
    v2 = 16.0
    v3 = 16.0
    Xw, Yw, Tp = kiwi.rotate_vel(v1, v2, v3, np.pi/4)

    # Forward
    v1 = -(np.sqrt(3)/2)*3
    v2 = 0.0
    v3 = (np.sqrt(3)/2)*3
    Xw, Yw, Tp = kiwi.movement_vel(v1, v2, v3, 2)  

    # Rotate left
    v1 = 16.0
    v2 = 16.0
    v3 = 16.0
    Xw, Yw, Tp = kiwi.rotate_vel(v1, v2, v3, np.pi/4)

    x = np.asarray(Xw)
    y = np.asarray(Yw)
    theta = np.asarray(Tp)

    x1, y1, x2, y2, x3, y3 = rotate_edges(x.copy(), y.copy(), theta.copy())


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


    for i, frame in enumerate(fig.frames):
        pio.write_image(fig.frames[i], f"/home/syed/myros_ws/src/kiwi_drive/scripts/images/frame_{i}.png", width=1920, height=1080, scale=1, engine="kaleido")

    images = [Image.open(f"/home/syed/myros_ws/src/kiwi_drive/scripts/images/frame_{n}.png") for n in range(N)]

    images[0].save('/home/syed/myros_ws/src/kiwi_drive/scripts/gifs/kiwi_kinematics_basic.gif', save_all=True, append_images=images[1:], duration=100, loop=0)
