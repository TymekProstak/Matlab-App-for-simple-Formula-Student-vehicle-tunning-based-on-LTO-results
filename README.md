# MATLAB App for Formula Student LTO-Based Vehicle Tuning

MATLAB App Designer project for simple Formula Student vehicle tuning based on LTO / minimum-lap-time optimization.

The app is built around a GUI workflow for loading a track, editing vehicle/solver parameters, generating an initial guess, solving an NLP with **CasADi + IPOPT**, plotting results, and exporting solution/config/plots.

Current working backend:

```text
Point-mass free-line NLP
```

Planned backend direction:

```text
Dynamic bicycle → quasi-static suspension → dynamic suspension
```

---

## 1. CasADi installation

The NLP solver uses **CasADi + IPOPT**.

Automatic installation:

```matlab
install_casadi("auto")
```

Forced OS selection:

```matlab
install_casadi("windows")
install_casadi("linux")
install_casadi("mac")
```

CasADi is installed locally into:

```text
external/
```

The `external/` folder should not be committed to Git.

---

## 2. How to run

Clone the repository:

```bash
git clone git@github.com:TymekProstak/Matlab-App-for-simple-Formula-Student-vehicle-tunning-based-on-LTO-results.git
cd Matlab-App-for-simple-Formula-Student-vehicle-tunning-based-on-LTO-results
```

Open MATLAB in the project root and run:

```matlab
startup
```

Launch the app:

```matlab
LTO_tunner_app
```

---

## 3. What the app does

The app provides a GUI workflow for:

- loading a Formula Student track from CSV,
- loading/saving config from JSON,
- editing vehicle, tire, suspension, drivetrain, bounds, cost and solver parameters,
- preparing a solver-ready track,
- generating a backward-forward initial guess,
- solving an NLP with CasADi/IPOPT,
- plotting solution and initial guess,
- exporting MAT, CSV folder, config JSON, track CSV and PNG plots.

General workflow:

```text
track CSV + GUI config
        ↓
track preprocessing
        ↓
optional backward-forward initial guess
        ↓
CasADi + IPOPT NLP solve
        ↓
common solution structure
        ↓
plots / MAT export / CSV export / output package
```

---

## 4. Track format

Expected CSV columns:

```text
x_left_m, y_left_m,
x_center_m, y_center_m,
x_right_m, y_right_m
```

The app converts the track to:

- path coordinate `s`,
- centerline heading,
- centerline curvature,
- left/right track width.

---

## 5. Current model: point-mass free-line NLP

The current implemented backend is a point-mass NLP in Frenet coordinates.

State vector:

$$
x =
\begin{bmatrix}
e_y \\
e_\psi \\
v_x \\
M_{\mathrm{cmd}} \\
M_{\mathrm{rear}} \\
t
\end{bmatrix}
$$

Control vector:

$$
u =
\begin{bmatrix}
\dot{M}_{\mathrm{cmd}} \\
\kappa_{\mathrm{vehicle}}
\end{bmatrix}
$$

Here, $\kappa_{\mathrm{vehicle}}$ is a pseudo-curvature decision variable, not a real steering angle.

Progress speed:

$$
v_s =
\frac{v_x \cos(e_\psi)}
{1-\kappa_{\mathrm{ref}}e_y}
$$

Frenet singularity protection:

$$
1-\kappa_{\mathrm{ref}}e_y \ge 0.2
$$

Rear longitudinal force:

$$
F_x =
\frac{M_{\mathrm{rear}}}{R}
$$

Longitudinal acceleration:

$$
a_x =
\frac{F_x - F_{\mathrm{drag}} - F_{\mathrm{rr}}}{m}
$$

with:

$$
F_{\mathrm{drag}} = C_d v_x^2
$$

$$
F_{\mathrm{rr}} = C_rmg
$$

Drivetrain first-order model:

$$
\dot{M}_{\mathrm{rear}} =
\frac{M_{\mathrm{cmd}}-M_{\mathrm{rear}}}{\tau}
$$

Lateral acceleration approximation:

$$
a_y = v_x^2\kappa_{\mathrm{vehicle}}
$$

Tire friction ellipse:

$$
\left(
\frac{F_{x,i}}{\mu_{x,i}F_{z,i}}
\right)^2
+
\left(
\frac{F_{y,i}}{\mu_{y,i}F_{z,i}}
\right)^2
\le 1
$$

Current normal load model:

```text
static load distribution + aerodynamic downforce
```

No dynamic mass transfer is included in the current point-mass backend.

Objective function:

$$
J =
t_N
+
q_{\dot M}\sum_k \dot M_{\mathrm{cmd},k}^2
+
q_\kappa\sum_k
\left(
\kappa_{\mathrm{vehicle},k+1}
-
\kappa_{\mathrm{vehicle},k}
\right)^2
-
q_v\sum_k v_{s,k}\frac{\Delta s_k}{L_{\mathrm{track}}}
$$

---

## 6. Backward-forward initial guess

The backward-forward solver is a fast initial-guess generator.

It estimates a speed profile from curvature and friction limits.

Basic relation:

$$
a_y = v_x^2\kappa
$$

so:

$$
v_{x,\max}
=
\sqrt{
\frac{a_{y,\max}}{|\kappa|}
}
$$

Then the algorithm applies forward acceleration and backward braking passes.

It is not the final optimizer. It only generates a useful seed for the NLP.

---

## 7. Planned model direction

### 7.1 Dynamic bicycle model

Planned as the first model with real lateral dynamics.

Assumptions:

- lateral velocity included,
- yaw rate included,
- no mass transfer,
- tire lateral force from SMF / MF5.2-like model,
- physical steering state instead of pseudo-curvature.

Typical equations:

$$
\dot v_y =
\frac{F_{yf}+F_{yr}}{m}
-
v_x r
$$

$$
\dot r =
\frac{l_fF_{yf}-l_rF_{yr}}{I_z}
$$

Slip angles:

$$
\alpha_f =
\delta -
\arctan
\left(
\frac{v_y+l_fr}{v_x}
\right)
$$

$$
\alpha_r =
-
\arctan
\left(
\frac{v_y-l_rr}{v_x}
\right)
$$

---

### 7.2 Quasi-static suspension model

Planned as a model with immediate load transfer.

Assumption:

```text
all load transfer is instantaneous / quasi-static / geometric
```

Longitudinal load transfer:

$$
\Delta F_{z,\mathrm{long}}
=
\frac{ma_xh_{\mathrm{CG}}}{L}
$$

Roll stiffness distribution:

$$
K_{\phi,F}
=
\frac{k_{w,F}t_f^2}{2}
+
K_{\mathrm{ARB},F}
$$

$$
K_{\phi,R}
=
\frac{k_{w,R}t_r^2}{2}
+
K_{\mathrm{ARB},R}
$$

$$
\lambda_\phi =
\frac{K_{\phi,F}}
{K_{\phi,F}+K_{\phi,R}}
$$

---

### 7.3 Dynamic suspension model

Planned as the most advanced model.

Assumption:

```text
total load transfer = geometric/direct part + elastic delayed part
```

The geometric part acts immediately.  
The elastic part is delayed with first-order dynamics:

$$
\dot{\Delta F}_{z,\mathrm{elastic}}
=
\frac{
\Delta F_{z,\mathrm{elastic,target}}
-
\Delta F_{z,\mathrm{elastic}}
}
{\tau}
$$

The natural frequency is estimated from wheel rate and sprung mass:

$$
f_n =
\frac{1}{2\pi}
\sqrt{
\frac{k_w}{m_s}
}
$$

The first-order time constant is estimated from frequency and damping ratio:

$$
\tau =
\frac{1}{2\pi \zeta f_n}
$$

In the future, this first-order elastic transfer may be replaced by second-order roll/pitch dynamics:

$$
I_\phi\ddot{\phi}
+
C_\phi\dot{\phi}
+
K_\phi\phi
=
M_\phi
$$

$$
I_\theta\ddot{\theta}
+
C_\theta\dot{\theta}
+
K_\theta\theta
=
M_\theta
$$

---

## 8. GUI overview

Main panels:

- **Top bar**: load track, load/save config, reset defaults, select model, solve.
- **Vehicle**: mass, geometry, inertia, aero, rolling resistance.
- **Tire**: tire radius, nominal load, lateral tire parameters, longitudinal friction limit.
- **Suspension**: roll centers, wheel rates, ARBs, sprung masses, damping, anti-dive, anti-squat.
- **Drivetrain**: torque limits, power limits, drivetrain time constant.
- **Solver**: model, solve level, initial guess strategy, discretization, IPOPT settings.
- **Bounds / Cost**: optimization limits and weights.
- **Files / Results**: paths, export options, plots and summary.

---

## 9. Export

Available export options:

```text
Export NLP solution to MAT
```

Saves `app.LastSolution` as a `.mat` file.

```text
Export NLP solution to CSV
```

Exports readable CSV files to a selected folder.

```text
Browse and export output package
```

Creates an output package with selected data, for example:

- NLP solution MAT,
- NLP CSV folder,
- config JSON,
- track CSV,
- initial guess MAT,
- backward-forward MAT,
- PNG plots.

---

## 10. Not implemented yet

Not implemented yet:

- dynamic bicycle backend,
- direct collocation integration,
- quasi-static suspension backend,
- dynamic suspension backend,
- complete initial guess vs NLP comparison tools.

---

## 11. Future work

Planned future extensions:

- add 4WD option,
- add hydraulic braking model,
- add accumulator energy usage model,
- add accumulator energy as state / decision variable,
- replace first-order elastic load transfer with second-order roll/pitch elastic transfer,
- improve solver diagnostics and sanity checks.

---

## 12. References

1. W. F. Milliken, D. L. Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995.  
   https://books.google.com/books/about/Race_Car_Vehicle_Dynamics.html?id=opgHfQzlnLEC

2. M. Massaro, D. J. N. Limebeer, *Minimum-lap-time optimisation and simulation*, Vehicle System Dynamics, 2021.  
   https://www.researchgate.net/publication/350662171_Minimum-lap-time_optimisation_and_simulation

---

## 13. Repository hygiene

Do not commit generated outputs or local dependencies.

Recommended ignored folders:

```text
external/
results/
```