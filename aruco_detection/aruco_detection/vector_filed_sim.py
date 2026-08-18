#!/usr/bin/env python3
"""Mo phong Vector Field Landing 2D - theo dung convention paper + guidance.cpp.

Convention (Figure 1b cua paper):
  x = phuong ngang (theo huong platform di chuyen)
  z = cao do
  phi = atan2(x, z)   <- goc lech khoi phuong THANG DUNG (khong phai atan2(z,x))
  phi_des = -27.5°    <- bisector nghieng ve phia drone tiep can

Vector field (guidance.cpp), voi convention phi=atan2(x,z):
  r = sqrt(x^2 + z^2)
  e_phi = phi - phi_des
  Vd_r   = -K1 * tanh(r/(eps1*r_max))       radial: keo ve landing point
  Vd_phi = -K2 * tanh(e_phi/(eps2*|dphi|))  tangential: keo ve bisector
  # Chuyen sang Descartes (voi phi do tu truc z):
  Vx = Vd_r*sin(phi) + Vd_phi*cos(phi)
  Vz = Vd_r*cos(phi) - Vd_phi*sin(phi)
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PHI_DES = np.radians(-27.5)
PHI_DELTA = np.radians(34.375)
R_MAX = 5.0
K1 = 0.25
K2 = 0.25
EPS1 = 8.5
EPS2 = 2.0
PHI_1 = PHI_DES + PHI_DELTA      # bien phai (gan thang dung)
PHI_2 = PHI_DES - PHI_DELTA      # bien trai (nghieng nhieu)


def vector_field(x, z):
    r = np.hypot(x, z)
    if r < 1e-6:
        return 0.0, 0.0
    phi = np.arctan2(x, z)           # goc tu truc thang dung
    e_phi = phi - PHI_DES

    K1n = K1 / np.tanh(1.0 / EPS1)
    denom = EPS2 * abs(PHI_DES - PHI_1)
    K2n = K2 / np.tanh(abs(PHI_DES) / denom)

    Xr = r / (EPS1 * R_MAX)
    Xphi = e_phi / denom

    Vd_r = -K1n * np.tanh(Xr)        # luon am -> giam r (ve landing)
    Vd_phi = -K2n * np.tanh(Xphi)    # keo e_phi ve 0 (ve bisector)

    # Chuyen sang Descartes voi phi do tu truc z
    Vx = Vd_r * np.sin(phi) + Vd_phi * np.cos(phi)
    Vz = Vd_r * np.cos(phi) - Vd_phi * np.sin(phi)
    return Vx, Vz


def simulate_trajectory(x0, z0, dt=0.05, steps=600):
    traj = [(x0, z0)]
    x, z = x0, z0
    for _ in range(steps):
        Vx, Vz = vector_field(x, z)
        x += Vx * dt
        z += Vz * dt
        traj.append((x, z))
        if np.hypot(x, z) < 0.08:
            break
    return np.array(traj)


def plot_field_and_trajectories():
    fig, ax = plt.subplots(figsize=(9, 7))

    L = R_MAX
    # Bien RDR + bisector (phi do tu truc z -> x=L*sin(phi), z=L*cos(phi))
    for pang in [PHI_1, PHI_2]:
        ax.plot([0, L*np.sin(pang)], [0, L*np.cos(pang)], 'g--', alpha=0.5, lw=1)
    ax.plot([0, L*np.sin(PHI_DES)], [0, L*np.cos(PHI_DES)], 'g-', alpha=0.8, lw=2,
            label='Bisector (an toan nhat)')

    # To mau RDR
    thetas = np.linspace(PHI_2, PHI_1, 30)
    xs = [0] + [L*np.sin(t) for t in thetas] + [0]
    zs = [0] + [L*np.cos(t) for t in thetas] + [0]
    ax.fill(xs, zs, color='green', alpha=0.08, label='RDR (vung detect duoc)')

    # Vector field grid
    xg = np.linspace(-4.0, 1.0, 24)
    zg = np.linspace(0.1, 4.5, 22)
    X, Z = np.meshgrid(xg, zg)
    U = np.zeros_like(X); W = np.zeros_like(Z)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            phi = np.arctan2(X[i,j], Z[i,j])
            if PHI_2 <= phi <= PHI_1:
                U[i,j], W[i,j] = vector_field(X[i,j], Z[i,j])
    ax.quiver(X, Z, U, W, color='steelblue', alpha=0.5, width=0.003, scale=6)

    # Quy dao tu 3 diem xuat phat
    starts = [(0.2, 2.0), (-1.0, 1.8), (-1.8, 1.4)]
    colors = ['red', 'orange', 'purple']
    for (x0, z0), c in zip(starts, colors):
        traj = simulate_trajectory(x0, z0)
        ax.plot(traj[:,0], traj[:,1], color=c, lw=2, label=f'Quy dao tu ({x0}, {z0})')
        ax.plot(x0, z0, 'x', color=c, markersize=10, mew=2)

    ax.plot(0, 0, 'k*', markersize=18, label='Landing point')
    ax.set_xlabel("x' (m) - phuong ngang")
    ax.set_ylabel("z (m) - cao do")
    ax.set_title("Vector Field Landing - drone hoi tu ve landing point (RDR)")
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-4.0, 1.5)
    ax.set_ylim(-0.2, 4.5)
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig("vector_field_landing.png", dpi=130)
    print("✅ Da luu hinh")


if __name__ == "__main__":
    print("=== Vector Field Landing (convention paper) ===")
    print(f"phi_des = {np.degrees(PHI_DES):.1f}° (bisector, do tu truc thang dung)")
    print(f"RDR: phi trong [{np.degrees(PHI_2):.1f}°, {np.degrees(PHI_1):.1f}°]\n")
    for x0, z0 in [(0.2, 2.0), (-1.0, 1.8), (-1.8, 1.4)]:
        traj = simulate_trajectory(x0, z0)
        f = traj[-1]; d = np.hypot(*f)
        print(f"Xuat phat ({x0:>5}, {z0}) -> ket thuc ({f[0]:.2f}, {f[1]:.2f}), "
              f"cach landing {d*100:.1f}cm ({len(traj)} buoc)")
    plot_field_and_trajectories()