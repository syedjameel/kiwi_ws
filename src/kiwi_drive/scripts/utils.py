import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objs as go


def rotate_edges(cx, cy, t):

    x1 = cx - 0.1
    y1 = cy + 0.1

    x2 = cx
    y2 = cy - 0.1

    x3 = cx + 0.1
    y3 = cy + 0.1

    x1_new = (x1-cx)*np.cos(t) - (y1-cy)*np.sin(t) + cx
    y1_new = (x1-cx)*np.sin(t) + (y1-cy)*np.cos(t) + cy

    x2_new = (x2-cx)*np.cos(t) - (y2-cy)*np.sin(t) + cx
    y2_new = (x2-cx)*np.sin(t) + (y2-cy)*np.cos(t) + cy

    x3_new = (x3-cx)*np.cos(t) - (y3-cy)*np.sin(t) + cx
    y3_new = (x3-cx)*np.sin(t) + (y3-cy)*np.cos(t) + cy

    return x1_new, y1_new, x2_new, y2_new, x3_new, y3_new


if __name__ == "__main__":
    N = 10

    cx = np.ones(N)
    cy = np.zeros(N)
    t = np.linspace(0, np.pi/2, N)

    x1, y1, x2, y2, x3, y3 = rotate_edges(cx, cy, t)

    print(cx, cy)
    print(x1, y1)
    print(x2, y2)
    print(x3, y3)

    plt.scatter(cx, cy)
    plt.scatter(x1, y1)
    plt.scatter(x2, y2)
    plt.scatter(x3, y3)

    plt.show()
    xm = np.min(cx) - 1.5
    xM = np.max(cx) + 1.5

    ym = np.min(cy) - 1.5
    yM = np.max(cy) + 1.5

    N = np.size(cx)

    # Create figure
    fig = go.Figure(
        data=[go.Scatter(x=cx, y=cy,
                        mode="lines",
                        line=dict(width=2, color="blue")),
            go.Scatter(x=cx, y=cy,
                        mode="lines",
                        line=dict(width=2, color="blue")),
            go.Scatter(x=cx, y=cy,
                        mode="lines",
                        line=dict(width=2, color="blue")),
            go.Scatter(x=cx, y=cy,
                        mode="lines",
                        line=dict(width=2, color="blue"))],
        layout=go.Layout(
            xaxis=dict(range=[xm, xM], autorange=False, zeroline=False),
            yaxis=dict(range=[ym, yM], autorange=False, zeroline=False),
            title_text="Kinematic Generation of a Planar Curve", hovermode="closest",
            updatemenus=[dict(type="buttons",
                            buttons=[dict(label="Play",
                                            method="animate",
                                            args=[None])])]),
        frames=[go.Frame(
            data=[go.Scatter(
                x=[cx[k]],
                y=[cy[k]],
                mode="markers",
                marker=dict(color="red", size=10,)),
                go.Scatter(
                x=[x1[k]],
                y=[y1[k]],
                mode="markers",
                marker=dict(color="blue", size=10,)),
                go.Scatter(
                x=[x2[k]],
                y=[y2[k]],
                mode="markers",
                marker=dict(color="yellow", size=10,)),
                go.Scatter(
                x=[x3[k]],
                y=[y3[k]],
                mode="markers",
                marker=dict(color="orange", size=10,))])

            for k in range(N)]
    )

    fig.show()