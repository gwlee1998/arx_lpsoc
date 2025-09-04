#!/usr/bin/env python3
import os, random, time

# -----------------------------
MAT_SIZE = 16
DIST = "uniform"
UMIN, UMAX = -100.0, 100.0
MU, SIGMA = 0.0, 0.5
OUT_X = "./trace/in_x.txt"
OUT_W = "./trace/in_w.txt"
# -----------------------------

def write_matrix(path, n, m, gen_float):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        for r in range(n):
            row = [f"{gen_float():.8e}" for _ in range(m)]
            f.write(" ".join(row) + "\n")

def main():
    # 실행할 때마다 다른 시드 (현재 시간 기반)
    random.seed(time.time())

    if DIST == "uniform":
        def gen(): return random.uniform(UMIN, UMAX)
    else:
        def gen(): return random.gauss(MU, SIGMA)

    write_matrix(OUT_X, MAT_SIZE, MAT_SIZE, gen)
    write_matrix(OUT_W, MAT_SIZE, MAT_SIZE, gen)
    print(f"Wrote {MAT_SIZE}x{MAT_SIZE} matrices to:\n  {OUT_X}\n  {OUT_W}")

if __name__ == "__main__":
    main()
