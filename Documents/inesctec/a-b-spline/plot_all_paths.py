import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.patches as patches

files = ["bspline.csv"] #"lazytheta.csv","cubic.csv","bezier.csv"
labels = ["Lazy Theta*","Cubic","B-Spline","Bezier"]
colors = ["black","red","blue","green"]

plt.figure(figsize=(6,4))

for fname, label, color in zip(files, labels, colors):
    try:
        df = pd.read_csv(fname)
        plt.plot(df["x"], df["y"], label=label, color=color)
    except FileNotFoundError:
        print(f"[AVISO] Ficheiro {fname} não encontrado.")

# Obstáculos (mesmo sítio que no C++)
obstacles = [
    (20, 20, 6, 6),  # centro x=20, y=20, largura 6, altura 6
    (40, 40, 6, 6)   # outro obstáculo, pode ajustar
]

for (cx, cy, w, h) in obstacles:
    rect = patches.Rectangle((cx-w/2, cy-h/2), w, h, linewidth=1, edgecolor='orange', facecolor='orange', alpha=0.5)
    plt.gca().add_patch(rect)

# Marcadores de Start e Goal
start_x, start_y = 10, 10
goal_x, goal_y = 50, 50

plt.scatter(start_x, start_y, s=100, c='green', marker='o', edgecolors='darkgreen', linewidths=2, label='Start', zorder=5)
plt.scatter(goal_x, goal_y, s=100, c='red', marker='s', edgecolors='darkred', linewidths=2, label='Goal', zorder=5)

plt.legend()
plt.xlim(0, 60)  # define o eixo X de 0 a 60
plt.ylim(0, 60)  # define o eixo Y de 0 a 60
plt.gca()  # para alinhar com grelha do C++
plt.gca()  # inverte o eixo X também
plt.title("Trajectories Comparison with Obstacles")

plt.savefig("paths.png", dpi=200)
print("Imagem salva em paths.png")
