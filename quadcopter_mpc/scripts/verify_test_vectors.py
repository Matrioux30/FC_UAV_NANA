"""Independent bit-exact model of the accelerator, built only from the exported ROM/format files.

Reads test_vectors/rom_<fmt>/*.mem + formats.txt and test_vectors/test_vectors_<fmt>.csv, recomputes g and z with
pure integer arithmetic, and checks every value matches. This is the behaviour the RTL must reproduce.
Usage: python verify_test_vectors.py [q16_16|q8_24] [--all]
"""
import os
import re
import sys
import numpy as np
import pandas as pd

NX, NU_TOTAL, NR, N_ITER_DEFAULT = 12, 20, 80, 80
INT32_MAX, INT32_MIN = 2**31 - 1, -2**31
HERE = os.path.dirname(os.path.abspath(__file__))


def sat(v):
    return max(INT32_MIN, min(INT32_MAX, v))


def from_wide(v, shift):
    return sat((v + (1 << (shift - 1))) >> shift)


def load_mem(path):
    vals = [int(line, 16) for line in open(path) if line.strip()]
    return [v - (1 << 32) if v & 0x80000000 else v for v in vals]


def load_rom(fmt):
    d = os.path.join(HERE, '..', 'test_vectors', f'rom_{fmt}')
    text = open(os.path.join(d, 'formats.txt')).read()
    frac = {k: int(re.search(rf'^{k}\s+Q\d+\.(\d+)', text, re.M).group(1)) for k in ('data', 'W', 'MX', 'MR', 'PARAMS')}
    n_iter = int(re.search(r'^N_ITER\s+(\d+)', text, re.M).group(1))
    rom = {k: load_mem(os.path.join(d, f'{k}.mem')) for k in ('W', 'MX', 'MR', 'ZMIN', 'ZMAX')}
    rom['P'] = load_mem(os.path.join(d, 'PARAMS_rho_alpha_1malpha.mem'))
    return frac, n_iter, rom


def accelerator(x, r, frac, n_iter, rom):
    W = [rom['W'][i * NU_TOTAL:(i + 1) * NU_TOTAL] for i in range(NU_TOTAL)]
    MX = [rom['MX'][i * NX:(i + 1) * NX] for i in range(NU_TOTAL)]
    MR = [rom['MR'][i * NR:(i + 1) * NR] for i in range(NU_TOTAL)]
    rho, alpha, oma = rom['P']
    fw, fx, fr, fp = frac['W'], frac['MX'], frac['MR'], frac['PARAMS']

    g = [sat(from_wide(sum(a * b for a, b in zip(MX[i], x)), fx) - from_wide(sum(a * b for a, b in zip(MR[i], r)), fr))
         for i in range(NU_TOTAL)]
    z = [0] * NU_TOTAL
    y = [0] * NU_TOTAL
    for _ in range(n_iter):
        rhs = [sat(from_wide(rho * sat(z[i] - y[i]), fp) - g[i]) for i in range(NU_TOTAL)]
        for i in range(NU_TOTAL):
            u = from_wide(sum(a * b for a, b in zip(W[i], rhs)), fw)
            u_hat = sat(from_wide(alpha * u, fp) + from_wide(oma * z[i], fp))
            cand = min(max(sat(u_hat + y[i]), rom['ZMIN'][i]), rom['ZMAX'][i])
            y[i] = sat(sat(y[i] + u_hat) - cand)
            z[i] = cand
    return g, z


def main():
    fmt = next((a for a in sys.argv[1:] if not a.startswith('--')), 'q16_16')
    frac, n_iter, rom = load_rom(fmt)
    tv = pd.read_csv(os.path.join(HERE, '..', 'test_vectors', f'test_vectors_{fmt}.csv'))
    if '--all' not in sys.argv:
        tv = tv.iloc[::25]
    xs = tv[[f'x{i}' for i in range(NX)]].values.astype(np.int64)
    rs = tv[[c for c in tv.columns if re.fullmatch(r'r\d+_[xyzw]', c)]].values.astype(np.int64)
    gs = tv[[f'g{i}' for i in range(NU_TOTAL)]].values
    zs = tv[[f'z{i}' for i in range(NU_TOTAL)]].values
    bad = 0
    for k, (x, r, g_exp, z_exp) in enumerate(zip(xs, rs, gs, zs)):
        g, z = accelerator([int(v) for v in x], [int(v) for v in r], frac, n_iter, rom)
        if list(g) != list(g_exp) or list(z) != list(z_exp):
            bad += 1
            if bad <= 3:
                print(f'MISMATCH cycle {tv.Cycle.iloc[k]}: max |dg| {np.abs(np.array(g) - g_exp).max()}, '
                      f'max |dz| {np.abs(np.array(z) - z_exp).max()} LSB')
    print(f'{fmt}: {len(tv) - bad}/{len(tv)} siklus bit-exact (g dan z), N_ITER = {n_iter}')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
