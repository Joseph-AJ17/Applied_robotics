[Home](../index.md)

# Forward Kinematics using DH Parameters (UR5e and KUKA KR16)

## 1. Introduction

- UR5e (6-axis collaborative robot)
- KUKA KR16 (6-axis industrial robot)

Forward kinematics computes the end-effector pose (position and orientation) with respect to the base frame, given:

- The robot geometric parameters (link lengths and offsets)
- The joint variables (joint angles for revolute joints)

The result is typically expressed as a homogeneous transformation matrix:

\[
{}^{0}T_{6}(\mathbf{q}) =
\begin{bmatrix}
{}^{0}R_{6}(\mathbf{q}) & {}^{0}p_{6}(\mathbf{q}) \\
0\;0\;0 & 1
\end{bmatrix}
\]

where:

- \( {}^{0}R_{6} \) is a 3×3 rotation matrix
- \( {}^{0}p_{6} \) is a 3×1 position vector
- \( \mathbf{q} = [q_1, q_2, q_3, q_4, q_5, q_6]^T \)

---

## 2. Model 1: UR5e forward kinematics (DH table)

### 2.1 Notes about the UR5e model

UR5e is a 6-DOF serial manipulator with a 3-DOF wrist. The DH table below uses symbolic geometric parameters \(L_1 \dots L_6\) and joint variables \(q_1 \dots q_6\).

Important modeling details:

- Angle offsets such as \(-\pi/2 + q_2\) are part of the frame assignment. They shift the zero position of a joint so that the DH frames align with a chosen “home” configuration.
- Length parameters \(L_i\) must be defined in consistent units (all in meters or all in millimeters).
- The resulting \( {}^{0}T_{6} \) depends on the chosen base frame and the end-effector frame definition (tool flange vs TCP).

### 2.2 Image placeholder (UR5e diagram)

Insert the UR5e kinematic diagram here (before the table):

![UR5e kinematic diagram](../recursos/imgs/Task_6/UR5E.jpeg)

### 2.3 UR5e DH table (LaTeX)

\[
\textbf{UR5e DH Parameters}=
\begin{array}{c|c|c|c|c}
\text{Link} & a_i & d_i & \alpha_i & \theta_i \\
\hline
1 & 0   & L_1 & \frac{\pi}{2}  & q_1 \\
2 & L_1 & 0   & 0                & -\frac{\pi}{2}+q_2 \\
3 & L_3 & 0   & 0                & q_3 \\
4 & 0   & L_4 & \frac{\pi}{2}  & -\frac{\pi}{2}+q_4 \\
5 & 0   & L_5 & -\frac{\pi}{2} & q_5 \\
6 & 0   & L_6 & 0                & -\pi+q_6
\end{array}
\]

### 2.4 How to compute the UR5e forward kinematics

1. Build each \( {}^{i-1}T_i \) using the matrix definition in Section 2.2 and the parameters from the table.
2. Substitute joint angles \(q_i\) (radians) and link constants \(L_i\).
3. Multiply all transforms in order to get \( {}^{0}T_{6} \).
4. Extract position and orientation from \( {}^{0}T_{6} \).
5. If you have a TCP/tool offset, multiply by \( {}^{6}T_{TCP} \).

---

## 3. Model 2: KUKA KR16 forward kinematics (DH-like table)

### 3.1 Notes about the KR16 model

KUKA KR16 is a 6-DOF industrial manipulator. Many KR-series robots are modeled with a DH (or DH-like) parameterization. The table below uses the symbols:

- \(L_i\): translational parameter (commonly used as an offset along the z-axis in DH-like models)
- \(D2_i\): translational parameter (commonly used as a distance along the x-axis in DH-like models)
- \(\theta_i\): joint angle (contains the variable \(q_i\) and possible constant offsets)
- \(\alpha_i\): twist angle between axes

This model is equivalent to the standard DH form if you interpret:

- \(d_i \leftarrow L_i\)
- \(a_i \leftarrow D2_i\)

and use:

\[
{}^{i-1}T_i = R_z(\theta_i)\,T_z(L_i)\,T_x(D2_i)\,R_x(\alpha_i)
\]

As with the UR5e, constant angle terms (for example \(\pi/2\) shifts) are tied to the chosen reference frames and the definition of the robot “zero” configuration.

### 3.2 Image placeholder (KR16 diagram)

Insert the KR16 kinematic diagram here (before the table):

![KUKA KR16 kinematic diagram](../recursos/imgs/Task_6/KUKAKR16.jpeg)


### 3.3 KR16 parameter table (LaTeX)

\[
\textbf{KUKA KR16 Parameters}=
\begin{array}{c|c|c|c|c}
\text{Link} & L_i & D2_i & \theta_i & \alpha_i \\
\hline
1 & -L_1      & -L_2 & q_1            & -\frac{\pi}{2} \\
2 & 0         & L_3  & \frac{\pi}{2}+q_2 & \pi \\
3 & 0         & 0    & q_3            & \frac{\pi}{2} \\
4 & L_4+L_5   & 0    & q_4            & -\frac{\pi}{2} \\
5 & 0         & 0    & q_5            & \frac{\pi}{2} \\
6 & -L_6      & 0    & q_6            & \pi
\end{array}
\]

### 3.4 How to compute the KR16 forward kinematics

1. Use the transform definition \( {}^{i-1}T_i = R_z(\theta_i)\,T_z(L_i)\,T_x(D2_i)\,R_x(\alpha_i) \).
2. Substitute link constants \(L_i\), \(D2_i\) and joint variables \(q_i\).
3. Multiply the six transforms in order to obtain \( {}^{0}T_{6} \).
4. Extract \( {}^{0}p_{6} \) and \( {}^{0}R_{6} \).
5. Apply any TCP/tool transform if needed.

---