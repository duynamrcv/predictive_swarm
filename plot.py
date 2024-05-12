from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from config import *

def getCircle(x,y,r):
    theta = np.linspace( 0 , 2 * np.pi , 150 )   
    a = x + r * np.cos( theta )
    b = y + r * np.sin( theta )
    return a, b

# time, x, y, z, vx, vy, vz, ux, uy, uz, d1, d2, d3
path0 = np.load("path_0.npy")
path1 = np.load("path_1.npy")
path2 = np.load("path_2.npy")
path3 = np.load("path_3.npy")
path4 = np.load("path_4.npy")

## Plot motion paths
fig = plt.figure(figsize=(6,3))
ax = fig.subplots()
ax.set_title("Drone path")
ax.grid(True)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')

# obstacles
for j in range(OBSTACLES.shape[0]):
    x, y, r = OBSTACLES[j,:]
    a, b = getCircle(x, y, r)
    ax.plot(a, b, '-k')
    a, b = getCircle(x, y, r+ROBOT_RADIUS)
    ax.plot(a, b, '--k')

ax.plot(path0[:,1], path0[:,2], label="Drone 0")
ax.plot(path1[:,1], path1[:,2], label="Drone 1")
ax.plot(path2[:,1], path2[:,2], label="Drone 2")
ax.plot(path3[:,1], path3[:,2], label="Drone 3")
ax.plot(path4[:,1], path4[:,2], label="Drone 4")
ax.axis('scaled')
plt.legend()
plt.title("Motion paths")
plt.tight_layout()
plt.savefig("results/path.png")

# Plot Speed
plt.figure(figsize=(6,3))
speeds = np.array([np.linalg.norm(path0[:,4:7], axis=1),
                   np.linalg.norm(path1[:,4:7], axis=1),
                   np.linalg.norm(path2[:,4:7], axis=1),
                   np.linalg.norm(path3[:,4:7], axis=1),
                   np.linalg.norm(path4[:,4:7], axis=1)]).T
plt.fill_between(path0[:,0], np.min(speeds,axis=1), np.max(speeds,axis=1), color="#1f77b4", label="Max/Min", alpha=0.3)
plt.plot(path0[:,0], np.mean(speeds,axis=1), 'b-', label="Average")
plt.plot([path0[0,0], path0[-1,0]], [VREF, VREF], 'g--', label="VREF") 
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.xlim([path0[0,0], path0[-1,0]])
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("results/speed.png")

# Plot distance
plt.figure(figsize=(6,3))

# distances = np.array([np.linalg.norm(path0[:,9:13], axis=1),
#                       np.linalg.norm(path1[:,9:13], axis=1),
#                       np.linalg.norm(path2[:,9:13], axis=1),
#                       np.linalg.norm(path3[:,9:13], axis=1),
#                       np.linalg.norm(path4[:,9:13], axis=1)]).T
distances = np.hstack([path0[:,10:13], path1[:,10:13], path2[:,10:13], path3[:,10:13], path4[:,10:13]])
plt.fill_between(path0[:,0], np.min(distances,axis=1), np.max(distances,axis=1), color="#1f77b4", label="Max/Min", alpha=0.3)
plt.plot(path0[:,0], np.mean(distances,axis=1), 'b-', label="Average")
plt.plot([path0[0,0], path0[-1,0]], [2*ROBOT_RADIUS, 2*ROBOT_RADIUS], 'k--', label="Safety radius")
plt.plot([path0[0,0], path0[-1,0]], [DREF, DREF], 'g--', label="VREF") 
plt.xlabel("Time (s)")
plt.ylabel("Inter-agent distance (m)")
plt.xlim([path0[0,0], path0[-1,0]])
plt.ylim([0, 1.6])
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("results/distance.png")

# Plot order
plt.figure(figsize=(6,3))
headings = []
for i in range(1,len(path0)):
    heading = path0[i,4:6]/np.linalg.norm(path0[i,4:6]) \
            + path1[i,4:6]/np.linalg.norm(path1[i,4:6]) \
            + path2[i,4:6]/np.linalg.norm(path2[i,4:6]) \
            + path3[i,4:6]/np.linalg.norm(path3[i,4:6]) \
            + path4[i,4:6]/np.linalg.norm(path4[i,4:6])
    headings.append(np.linalg.norm(heading)/NUM_UAV)
plt.plot(path0[1:,0], headings, '-b')
plt.xlabel("Time (s)")
plt.ylabel("Order")
plt.xlim([path0[0,0], path0[-1,0]])
plt.ylim([0, 1.1])
plt.grid(True)
plt.tight_layout()
plt.savefig("results/order.png")

plt.show()