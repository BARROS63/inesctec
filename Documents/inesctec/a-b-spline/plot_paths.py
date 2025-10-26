import matplotlib.pyplot as plt
import pandas as pd

files = ["lazytheta.csv","cubic.csv","bspline.csv","bezier.csv"]
labels = ["Lazy Theta*","Cubic","B-Spline","Bezier"]
colors = ["black","red","blue","green"]

plt.figure(figsize=(6,4))
for fname, label, color in zip(files, labels, colors):
    df = pd.read_csv(fname)
    plt.plot(df["x"], df["y"], label=label, color=color)

plt.legend()
plt.gca().invert_yaxis()  # porque o grid do C++ tem origem no canto
plt.title("Trajectories Comparison")
plt.show()
