# Robotics Class Project 3 – Kinematics Toolkit

> Forward, inverse, and velocity kinematics utilities for a course project in robot kinematics.

This project contains small, focused Python scripts for computing:
- **Forward Kinematics (FK)**
- **Inverse Kinematics (IK)**
- **Velocity Kinematics / Jacobians**
- A documented **symbolic IK attempt** (for learning/reference)

It’s designed to be easy to run from the command line for quick checks or homework write-ups, and simple to read for classmates or recruiters skimming your code.

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

## Notes & tips

- **Symbolic IK**: The `FailedAttempt_SymbolicInverseKinematics.py` file shows the algebraic direction explored (useful for reports and for understanding where closed-form gets sticky).
- **Numeric stability**: If IK oscillates or fails to converge, try:
  - Better initial guesses
  - Slightly relaxed tolerances
  - Damped least-squares (Levenberg-Marquardt style) parameters
- **Units**: Be consistent—stick to meters and radians throughout.

---

## Reproducing the report

For context and grading criteria, see the assignment brief and the final write-up included in the repo:
- `Project3-instructions.pdf`
- `Final Report (charles friley).pdf`

---

## FAQ

**Q: I get import errors for `roboticstoolbox` or `spatialmath`.**  
A: Double-check you’re inside the virtual environment and installed packages with the exact names shown above.

**Q: Can I swap in a different robot?**  
A: Yes—edit the DH parameters and joint limits in each script. FK/IK/Jacobian code is intentionally minimal to make that easy.

---

## License

If you need one, consider adding MIT here. (Currently, the repo has no license file.)

---

## Acknowledgments

Course staff and the Python robotics ecosystem (`roboticstoolbox-python`, `spatialmath-python`, `numpy`, `sympy`).
