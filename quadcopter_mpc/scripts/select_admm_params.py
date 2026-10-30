"""Grid search for ADMM (rho, alpha, N_ITER) on QPs sampled from the golden-model closed loop.

Accuracy metric: max |u0_admm - u0_exact| / u_max over every cycle in test_vectors.csv, with u0_exact from
OSQP (polished, eps 1e-10). Requires test_vectors.csv from main_golden_model.exe.
"""
import os
import numpy as np
import pandas as pd
import mpc_model as m

TARGET = 1e-3  # 0.1 % of actuator limit
HERE = os.path.dirname(os.path.abspath(__file__))


def load_problems(prob):
    tv = pd.read_csv(os.path.join(HERE, '..', 'test_vectors', 'test_vectors.csv'))
    xs = tv[[f'x{i}' for i in range(m.NX)]].values
    rs = tv[[f'r{p}_{c}' for p in range(m.NP) for c in 'xyzw']].values
    gs = xs @ prob['Mxs'].T - rs @ prob['Mrs'].T
    exact = np.array([m.solve_osqp(prob['Hs'], g, prob['z_min'], prob['z_max']) for g in gs])
    return tv['Scenario'].values, gs, exact


def evaluate(prob, scen, gs, exact, rho_unit, alpha, n_iter):
    p = dict(prob)
    p['rho'] = rho_unit * prob['rho'] / m.RHO
    p['W'] = np.linalg.inv(prob['Hs'] + p['rho'] * np.eye(m.NU_TOTAL))
    scale = prob['E'][:m.NU] / prob['u_max'][:m.NU]
    errs = np.array([np.abs((m.admm_scaled(p, g, n_iter, alpha) - z)[:m.NU] * scale).max()
                     for g, z in zip(gs, exact)])
    return {s: errs[scen == s].max() for s in dict.fromkeys(scen)}


def main():
    prob = m.build_scaled_problem()
    scen, gs, exact = load_problems(prob)
    print(f'{len(gs)} QPs, target worst-case error {TARGET:g} of u_max')
    for alpha in (1.0, 1.6):
        for rho in (0.1, 0.15, 0.2, 0.25, 0.3):
            for n in (30, 40, 60, 80):
                r = evaluate(prob, scen, gs, exact, rho, alpha, n)
                worst = max(r.values())
                tag = ' <= target' if worst <= TARGET else ''
                print(f'alpha={alpha} rho={rho:<5} N={n:<3} worst={worst:.2e}  ' +
                      '  '.join(f'{s}={v:.1e}' for s, v in r.items()) + tag)
    print(f'\nDeployed: rho={m.RHO}, alpha={m.ALPHA}, N_ITER={m.N_ITER}')


if __name__ == '__main__':
    main()
