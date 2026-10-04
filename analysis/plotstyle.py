"""One consistent, colour-blind-safe look for every figure (Okabe-Ito based)."""
import matplotlib.pyplot as plt

C = dict(blue="#0072B2", orange="#E69F00", green="#009E73", red="#D55E00", purple="#CC79A7",
         sky="#56B4E9", grey="#8C8C8C", dark="#222222", band="#E8F1F8", warn="#D55E00")


def style():
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
        "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.25,
        "grid.linewidth": 0.6, "axes.edgecolor": "#444444", "legend.frameon": False,
        "figure.facecolor": "white", "savefig.facecolor": "white",
    })
