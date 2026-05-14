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

def load_csv(path):

    df = pd.read_csv(
        path,
        header=None,
        dtype=str
    )

    df = df.apply(
        pd.to_numeric,
        errors='coerce'
    )

    df = df.dropna()

    phi0 = df.iloc[:, 0].values.astype(np.float64)
    phi1 = df.iloc[:, 1].values.astype(np.float64)
    phi2 = df.iloc[:, 2].values.astype(np.float64)
    phi3 = df.iloc[:, 3].values.astype(np.float64)

    y = -phi0
    u =  phi2

    return u, y

def crop_signal(u, y):

    idx = np.where(np.abs(u) > 1e-3)[0]

    if len(idx) == 0:
        return u, y

    start = idx[0]
    end = idx[-1]

    return u[start:end], y[start:end]

def identify_second_order(u, y):

    N = len(y)

    Phi = []
    Y = []

    for k in range(2, N):

        Phi.append([
            -y[k - 1],
            -y[k - 2],
             u[k - 1],
             u[k - 2]
        ])

        Y.append(y[k])

    Phi = np.array(Phi)
    Y = np.array(Y)

    theta = np.linalg.lstsq(Phi, Y, rcond=None)[0]

    a1, a2, b1, b2 = theta

    num = [0, b1, b2]
    den = [1, a1, a2]

    return theta, num, den

def analyze_system(num, den):

    z, p, k = signal.tf2zpk(num, den)

    w, h = signal.freqz(num, den, worN=4096, fs=FS)

    mag = np.abs(h)

    mag /= np.max(mag)

    idx = np.where(mag <= 0.707)[0]

    if len(idx) > 0:
        fc = w[idx[0]]
    else:
        fc = -1

    return z, p, fc, w, mag

def save_plot(path, u, y, w, mag):

    t = np.arange(len(u)) * TS

    plt.figure(figsize=(12, 5))
    plt.plot(t, u, label='input')
    plt.plot(t, y, label='output')
    plt.legend()
    plt.xlabel("time")
    plt.grid()

    plt.savefig(os.path.join(path, "waveform.png"))
    plt.close()

    plt.figure(figsize=(12, 5))
    plt.semilogx(w, 20*np.log10(mag))
    plt.xlabel("Hz")
    plt.ylabel("dB")
    plt.grid()

    plt.savefig(os.path.join(path, "bode.png"))
    plt.close()

def save_report(path,
                theta,
                z,
                p,
                fc):

    report = {}

    report["a1"] = float(theta[0])
    report["a2"] = float(theta[1])
    report["b1"] = float(theta[2])
    report["b2"] = float(theta[3])

    report["zeros"] = [complex(v).__str__() for v in z]
    report["poles"] = [complex(v).__str__() for v in p]

    report["cutoff_frequency_hz"] = float(fc)

    with open(os.path.join(path, "report.json"), "w") as f:
        json.dump(report, f, indent=4)

    with open(os.path.join(path, "report.txt"), "w") as f:

        f.write("Second Order Identification\n\n")

        f.write(f"a1 = {theta[0]:.6f}\n")
        f.write(f"a2 = {theta[1]:.6f}\n")
        f.write(f"b1 = {theta[2]:.6f}\n")
        f.write(f"b2 = {theta[3]:.6f}\n\n")

        f.write(f"Cutoff Frequency = {fc:.3f} Hz\n\n")

        f.write("Zeros:\n")
        for v in z:
            f.write(f"{v}\n")

        f.write("\nPoles:\n")
        for v in p:
            f.write(f"{v}\n")

def process(csv_name, out_name):

    path = os.path.join(RESULT_DIR, out_name)

    os.makedirs(path, exist_ok=True)

    u, y = load_csv(
        os.path.join(DATA_DIR, csv_name)
    )

    u, y = crop_signal(u, y)

    theta, num, den = identify_second_order(u, y)

    z, p, fc, w, mag = analyze_system(num, den)

    save_plot(path, u, y, w, mag)

    save_report(path,
                theta,
                z,
                p,
                fc)

if __name__ == "__main__":

    process("LK_yaw.csv", "yaw")

    process("LK_pitch.csv", "pitch")