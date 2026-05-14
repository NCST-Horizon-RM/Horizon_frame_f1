import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal

FS = 1000.0
TS = 1.0 / FS

DATA_DIR = "data"
RESULT_DIR = "result"

os.makedirs(RESULT_DIR, exist_ok=True)


# ---------------------------
# 数据读取 + 清洗（关键修复）
# ---------------------------
def load_csv(path):

    df = pd.read_csv(path, header=None, dtype=str)
    df = df.apply(pd.to_numeric, errors='coerce')
    df = df.dropna()

    Phi = df.iloc[:, 0:4].values.astype(np.float64)
    y = df.iloc[:, 4].values.astype(np.float64)

    return Phi, y


# ---------------------------
# 标准化（稳定核心）
# ---------------------------
def normalize(Phi, y):

    Phi = (Phi - np.mean(Phi, axis=0)) / (np.std(Phi, axis=0) + 1e-8)
    y = (y - np.mean(y)) / (np.std(y) + 1e-8)

    return Phi, y


# ---------------------------
# RLS / LS（加正则防爆）
# ---------------------------
def identify(Phi, y):

    lam = 1e-3

    A = Phi.T @ Phi + lam * np.eye(Phi.shape[1])
    b = Phi.T @ y

    theta = np.linalg.solve(A, b)

    a1, a2, b1, b2 = theta

    num = [0.0, b1, b2]
    den = [1.0, a1, a2]

    return theta, num, den


# ---------------------------
# 频响分析（稳定版）
# ---------------------------
def analyze(num, den):

    z, p, k = signal.tf2zpk(num, den)

    w, h = signal.freqz(num, den, worN=4096, fs=FS)

    mag = np.abs(h)

    mag = np.nan_to_num(mag, nan=0.0, posinf=0.0, neginf=0.0)

    mx = np.max(mag)
    if mx > 1e-9:
        mag = mag / mx

    idx = np.where(mag <= 0.707)[0]
    fc = w[idx[0]] if len(idx) > 0 else -1

    return z, p, fc, w, mag


# ---------------------------
# 画图（修复 u bug）
# ---------------------------
def save_plot(path, u, y, w, mag):

    t = np.arange(len(y)) * TS

    plt.figure()
    plt.plot(t, u, label="u")
    plt.plot(t, y, label="y")
    plt.legend()
    plt.grid()
    plt.title("Time Response")
    plt.savefig(os.path.join(path, "waveform.png"))
    plt.close()

    plt.figure()
    plt.semilogx(w, 20 * np.log10(mag + 1e-9))
    plt.grid()
    plt.title("Bode Magnitude")
    plt.savefig(os.path.join(path, "bode.png"))
    plt.close()


# ---------------------------
# 报告
# ---------------------------
def save_report(path, theta, z, p, fc):

    report = {
        "a1": float(theta[0]),
        "a2": float(theta[1]),
        "b1": float(theta[2]),
        "b2": float(theta[3]),
        "cutoff_hz": float(fc),
        "zeros": [str(v) for v in z],
        "poles": [str(v) for v in p],
    }

    with open(os.path.join(path, "report.json"), "w") as f:
        json.dump(report, f, indent=4)

    with open(os.path.join(path, "report.txt"), "w") as f:
        f.write("SYSTEM IDENTIFICATION REPORT\n\n")
        f.write(f"a1={theta[0]:.6f}\n")
        f.write(f"a2={theta[1]:.6f}\n")
        f.write(f"b1={theta[2]:.6f}\n")
        f.write(f"b2={theta[3]:.6f}\n\n")
        f.write(f"Cutoff={fc:.3f} Hz\n\n")
        f.write("Poles:\n")
        for i in p:
            f.write(str(i) + "\n")


# ---------------------------
# 主流程
# ---------------------------
def process(file, name):

    path = os.path.join(RESULT_DIR, name)
    os.makedirs(path, exist_ok=True)

    Phi, y = load_csv(os.path.join(DATA_DIR, file))

    Phi, y = normalize(Phi, y)

    theta, num, den = identify(Phi, y)

    z, p, fc, w, mag = analyze(num, den)

    u = Phi[:, 2]   # 修复 u bug

    save_plot(path, u, y, w, mag)
    save_report(path, theta, z, p, fc)


if __name__ == "__main__":

    process("LK_yaw.csv", "yaw")
    process("LK_pitch.csv", "pitch")