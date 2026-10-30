"""Closed-loop check of the deployed MPC on the linear prediction model (fast sanity check before C++)."""
import numpy as np
import mpc_model as m


def run(x0, ref_fn, steps=1000):
    prob = m.build_scaled_problem()
    Ad, Bd = prob['Ad'], prob['Bd']
    x = x0.copy()
    err = []
    for k in range(steps):
        t = k * m.TS
        r = np.array([ref_fn(t + p * m.TS) for p in range(m.NP)])
        err.append(r[0, :3] - x[:3])
        z = m.admm_scaled(prob, prob['Mxs'] @ x - prob['Mrs'] @ r.ravel())
        x = Ad @ x + Bd @ (prob['E'][:m.NU] * z[:m.NU])
    return x, np.sqrt(np.mean(np.square(err), axis=0))


def main():
    x_end, _ = run(np.zeros(12), lambda t: [0, 0, 0.0 if t < 1.0 else 1.5, 0])
    print(f"Step 5 s, final z: {x_end[2]:.4f} m (target 1.5 m)")
    x0 = np.zeros(12)
    x0[0], x0[2] = 0.5, 1.0
    _, rmse = run(x0, lambda t: [0.5 * np.cos(0.5 * t), 0.5 * np.sin(0.5 * t), 1 + 0.2 * np.sin(t), 0])
    print(f"Trajectory 5 s RMSE x/y/z: {rmse[0]:.4f} / {rmse[1]:.4f} / {rmse[2]:.4f} m")


if __name__ == '__main__':
    main()
