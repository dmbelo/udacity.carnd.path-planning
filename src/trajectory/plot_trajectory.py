import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df_map = pd.read_table("../data/highway_map.csv", names=["x", "y", "s", "dx", "dy"], sep=" ")
df_traj = pd.read_table("output.csv", names=["x", "y"], sep=",")

plt.figure() 
plt.plot(df_map.x, df_map.y, "-s", df_traj.x, df_traj.y, "o")
plt.xlabel("x")
plt.ylabel("y")
plt.legend(["Map", "Trajectory"])

plt.show()