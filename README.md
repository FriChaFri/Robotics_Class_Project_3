# Robotics Class Project 3 – Kinematics Toolkit

> Forward, inverse, and velocity kinematics utilities for a course project in robot kinematics.

This project contains small, focused Python scripts for computing:
- **Forward Kinematics (FK)**
- **Inverse Kinematics (IK)**
- **Velocity Kinematics / Jacobians**
- A documented **symbolic IK attempt** (for learning/reference)

---

## Repo layout

- `ForwardKinematics.py` – forward-kinematics utilities and example usage.  
- `InverseKinematics.py` – inverse-kinematics routines and example usage.  
- `VelocityKinematics.py` – velocity kinematics / Jacobian computations.  
- `FailedAttempt_SymbolicInverseKinematics.py` – an exploratory (and intentionally “failed”) symbolic IK approach with notes.  
- `Project3-instructions.pdf` – original assignment brief.  
- `Final Report (charles friley).pdf` – write-up summarizing method and results.  

---

## Requirements

- **Python 3.12.8** (or 3.10+ should also work)  
- Python packages:
  - `numpy<2`
  - `spatialmath-python`
  - `roboticstoolbox-python`
  - `sympy`

Install in a fresh virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
python -m pip install --upgrade pip
pip install "numpy<2" spatialmath-python roboticstoolbox-python sympy
```

---

## Quick start

### 1) Forward kinematics
```bash
python ForwardKinematics.py
```
- Expected behavior: prints or plots end-effector pose(s) for the configured joint values.
- Tip: adjust joint values / link parameters in the script to match your manipulator.

### 2) Inverse kinematics
```bash
python InverseKinematics.py
```
- Expected behavior: solves for joint angles that achieve a target pose (if a solution exists), then verifies by recomputing FK.

### 3) Velocity kinematics / Jacobian
```bash
python VelocityKinematics.py
```
- Expected behavior: computes the manipulator Jacobian and maps joint rates to end-effector spatial velocity.

---

## Configuration

Most scripts define **DH parameters**, **link lengths**, and **joint limits** at the top of the file. Edit those to match your mechanism. If your robot differs (e.g., planar vs. spatial, revolute vs. prismatic), mirror that in the DH table and the joint type flags.

---

## Acknowledgments

Course staff and the Python robotics ecosystem (`roboticstoolbox-python`, `spatialmath-python`, `numpy`, `sympy`).
