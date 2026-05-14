import json
import numpy as np
from scipy.linalg import solve_discrete_are

FS = 1000.0
TS = 1.0 / FS

def dlqr(A, B, Q, R):

    P = solve_discrete_are(A, B, Q, R)

    K = np.linalg.inv(
        B.T @ P @ B + R
    ) @ (B.T @ P @ A)

    return K, P

def load_model(path):

    with open(path, "r") as f:
        data = json.load(f)

    a1 = data["a1"]
    a2 = data["a2"]

    b1 = data["b1"]
    b2 = data["b2"]

    return a1, a2, b1, b2

def build_state_space(a1, a2, b1, b2):

    A = np.array([
        [a1, a2, b1],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ])

    B = np.array([
        [b2],
        [0.0],
        [1.0]
    ])

    C = np.array([[1.0, 0.0, 0.0]])
    D = np.array([[0.0]])
    
    return A, B, C, D

def design(name):

    path = f"result/{name}/report.json"

    a1, a2, b1, b2 = load_model(path)

    A, B, C, D = build_state_space(
        a1,
        a2,
        b1,
        b2
    )

    Q = np.diag([10, 1, 1])
    R = np.array([[1]])

    K, P = dlqr(A, B, Q, R)

    out = f"result/{name}/lqr_report.txt"

    with open(out, "w") as f:

        f.write("LQR DESIGN REPORT\n\n")

        f.write("A:\n")
        f.write(str(A))
        f.write("\n\n")

        f.write("B:\n")
        f.write(str(B))
        f.write("\n\n")

        f.write("K:\n")
        f.write(str(K))
        f.write("\n\n")

        eig = np.linalg.eigvals(A - B @ K)

        f.write("Closed Loop Poles:\n")

        for v in eig:
            f.write(f"{v}\n")

if __name__ == "__main__":

    design("yaw")

    design("pitch")