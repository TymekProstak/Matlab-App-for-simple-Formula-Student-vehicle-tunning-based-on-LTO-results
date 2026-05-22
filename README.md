# MATLAB App for Formula Student LTO-Based Vehicle Tuning

MATLAB App Designer project for simple Formula Student vehicle tuning based on **LTO / minimum-lap-time optimization**.

The app provides a GUI workflow for:
- loading a track,
- editing vehicle and solver parameters,
- generating an initial guess,
- solving an NLP with **CasADi + IPOPT**,
- plotting results,
- exporting solution/config/results.

## Current working backends

```text
Point-mass free-line NLP
Dynamic bicycle NLP
```

## Planned backend direction

```text
Quasi-static suspension → dynamic suspension
```

---

## 1. CasADi installation

The NLP solver uses **CasADi + IPOPT**.

Automatic installation:

```matlab
install_casadi("auto")
```

Forced operating-system selection:

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
git clone https://github.com/TymekProstak/Matlab-App-for-simple-Formula-Student-vehicle-tunning-based-on-LTO-results.git
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
- plotting the solution and initial guess,
- exporting MAT files, CSV folders, config JSON, track CSV and PNG plots.

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

---

## 5. Current model: point-mass free-line NLP

The current implemented backend is a point-mass NLP in Frenet coordinates.

State vector:

```math
x=\begin{bmatrix}e_y & e_\psi & v_x & M_{\mathrm{cmd}} & M_{\mathrm{rear}} & t\end{bmatrix}^{T}
```

Control vector:

```math
u=\begin{bmatrix}\dot{M}_{\mathrm{cmd}} & \kappa_{\mathrm{vehicle}}\end{bmatrix}^{T}
```

Here, \(\kappa_{\mathrm{vehicle}}\) is a pseudo-curvature decision variable, not a real steering angle.

Progress speed:

```math
v_s=\frac{v_x\cos(e_\psi)}{1-\kappa_{\mathrm{ref}}e_y}
```

Frenet singularity protection:

```math
1-\kappa_{\mathrm{ref}}e_y\ge 0.2
```

Rear longitudinal force:

```math
F_x=\frac{M_{\mathrm{rear}}}{R}
```

Longitudinal acceleration:

```math
a_x=\frac{F_x-F_{\mathrm{drag}}-F_{\mathrm{rr}}}{m}
```

Aerodynamic drag and rolling resistance:

```math
F_{\mathrm{drag}}=C_d v_x^2
```

```math
F_{\mathrm{rr}}=C_r m g
```

Drivetrain first-order model:

```math
\dot{M}_{\mathrm{rear}}=\frac{M_{\mathrm{cmd}}-M_{\mathrm{rear}}}{\tau}
```

Lateral acceleration approximation:

```math
a_y=v_x^2\kappa_{\mathrm{vehicle}}
```

Tire friction ellipse:

```math
\left(\frac{F_{x,i}}{\mu_{x,i}F_{z,i}}\right)^2+\left(\frac{F_{y,i}}{\mu_{y,i}F_{z,i}}\right)^2\le 1
```

Current normal load model:

```text
static load distribution + aerodynamic downforce
```

No dynamic mass transfer is included in the current point-mass backend.

Objective function:

```math
J=t_N+J_{\dot{M}}+J_{\kappa}-J_v
```

Torque-rate penalty:

```math
J_{\dot{M}}=q_{\dot{M}}\sum_k \dot{M}_{\mathrm{cmd},k}^{2}
```

Pseudo-curvature smoothness penalty:

```math
J_{\kappa}=q_{\kappa}\sum_k\left(\kappa_{\mathrm{vehicle},k+1}-\kappa_{\mathrm{vehicle},k}\right)^2
```

Progress-speed reward:

```math
J_v=q_v\sum_k v_{s,k}\frac{\Delta s_k}{L_{\mathrm{track}}}
```

---

## 6. Current model: dynamic bicycle NLP

The dynamic bicycle backend is a dynamic lateral vehicle model in Frenet coordinates.

State vector:

```math
x=\begin{bmatrix}e_y & e_\psi & v_x & v_y & r & M_{\mathrm{cmd}} & M_{\mathrm{rear}} & \delta_{\mathrm{cmd}} & \delta & \dot{\delta} & t\end{bmatrix}^{T}
```

Control vector:

```math
u=\begin{bmatrix}\dot{M}_{\mathrm{cmd}} & \dot{\delta}_{\mathrm{cmd}}\end{bmatrix}^{T}
```

The model includes:
- lateral velocity,
- yaw rate,
- physical steering angle,
- rear-wheel-drive longitudinal force,
- first-order drivetrain model,
- second-order steering actuator model,
- static normal loads with aerodynamic downforce,
- axle-level tire friction ellipse,
- no dynamic mass transfer.

Progress speed:

```math
v_s=\frac{v_x\cos(e_\psi)-v_y\sin(e_\psi)}{1-\kappa_{\mathrm{ref}}e_y}
```

Frenet singularity protection:

```math
1-\kappa_{\mathrm{ref}}e_y\ge 0.2
```

Rear longitudinal force:

```math
F_{x,R}=\frac{M_{\mathrm{rear}}}{R}
```

Dynamic bicycle equations:

```math
\dot{v}_x=\frac{F_{x,R}-F_{yf}\sin(\delta)-F_{\mathrm{drag}}-F_{\mathrm{rr}}}{m}+v_y r
```

```math
\dot{v}_y=\frac{F_{yf}\cos(\delta)+F_{yr}}{m}-v_x r
```

```math
\dot{r}=\frac{l_fF_{yf}\cos(\delta)-l_rF_{yr}}{I_z}
```

Slip angles:

```math
\alpha_f=\delta-\arctan\left(\frac{v_y+l_f r}{v_x}\right)
```

```math
\alpha_r=-\arctan\left(\frac{v_y-l_r r}{v_x}\right)
```

The lateral tire force is computed from an MF5.2-like tire model.

Axle-level tire friction ellipse:

```math
\left(\frac{F_{x,A}}{\mu_{x,A}F_{z,A}}\right)^2+\left(\frac{F_{y,A}}{\mu_{y,A}F_{z,A}}\right)^2\le 1
```

where \(A\) is the front or rear axle.

Current normal load model:

```text
static load distribution + aerodynamic downforce
```

No dynamic mass transfer is included in the current dynamic bicycle backend.

Drivetrain first-order model:

```math
\dot{M}_{\mathrm{rear}}=\frac{M_{\mathrm{cmd}}-M_{\mathrm{rear}}}{\tau}
```

Steering actuator model:

```math
\dot{\delta}_{\mathrm{cmd}}=u_{\delta}
```

```math
\dot{\delta}=\dot{\delta}
```

```math
\ddot{\delta}=\omega_n^2(\delta_{\mathrm{cmd}}-\delta)-2\zeta\omega_n\dot{\delta}
```

Objective function:

```math
J=t_N+J_{\dot{M}}+J_{\dot{\delta}}+J_{\beta}
```

Torque-rate penalty:

```math
J_{\dot{M}}=q_{\dot{M}}\sum_k \dot{M}_{\mathrm{cmd},k}^{2}
```

Steering command-rate penalty:

```math
J_{\dot{\delta}}=q_{\dot{\delta}}\sum_k \dot{\delta}_{\mathrm{cmd},k}^{2}
```

Beta consistency penalty:

```math
J_{\beta}=q_{\beta}\sum_k(\beta_{\mathrm{dyn},k}-\beta_{\mathrm{kin},k})^2\,\Delta s_k
```

Dynamic beta:

```math
\beta_{\mathrm{dyn}}=\arctan\left(\frac{v_y}{v_x}\right)
```

Kinematic beta:

```math
\beta_{\mathrm{kin}}=\arctan\left(\frac{l_r}{L}\tan(\delta)\right)
```

The beta cost does not directly penalize beta. It penalizes the difference between dynamic beta and kinematic beta.

---

## 7. Backward-forward initial guess

The backward-forward solver is a fast initial-guess generator.

It estimates a speed profile from curvature and friction limits.

Basic relation:

```math
a_y=v_x^2\kappa
```

Therefore:

```math
v_{x,\max}=\sqrt{\frac{a_{y,\max}}{|\kappa|}}
```

Then the algorithm applies forward acceleration and backward braking passes.

It is not the final optimizer. It only generates a useful seed for the NLP.

---

## 8. Planned model direction

### 8.1 Quasi-static suspension model

Planned as a model with immediate load transfer.

Main assumption:

```text
all load transfer is instantaneous / quasi-static
```

The wheel rates `k_w,F` and `k_w,R` are defined per one wheel.

Longitudinal load transfer:

```math
\Delta F_{z,\mathrm{long}} =
\frac{m a_x h_{\mathrm{CG}}}{L}
```

where:

```math
L = l_f + l_r
```

Roll axis height at the CG longitudinal position:

```math
h_{\mathrm{RA}} =
\frac{l_r}{L} h_{\mathrm{RC},F}
+
\frac{l_f}{L} h_{\mathrm{RC},R}
```

Global lateral direct/geometric and elastic fractions:

```math
\eta_{\mathrm{lat,geo}} =
\frac{h_{\mathrm{RA}}}{h_{\mathrm{CG}}}
```

```math
\eta_{\mathrm{lat,elastic}} =
1 -
\eta_{\mathrm{lat,geo}}
=
\frac{h_{\mathrm{CG}}-h_{\mathrm{RA}}}{h_{\mathrm{CG}}}
```

Direct/geometric lateral load transfer:

```math
\Delta F_{z,\mathrm{lat,geo},F} =
\frac{F_{y,F} h_{\mathrm{RC},F}}{t_f}
```

```math
\Delta F_{z,\mathrm{lat,geo},R} =
\frac{F_{y,R} h_{\mathrm{RC},R}}{t_r}
```

Roll stiffness distribution:

```math
K_{\phi,F} =
\frac{k_{w,F} t_f^2}{2}
+
K_{\mathrm{ARB},F}
```

```math
K_{\phi,R} =
\frac{k_{w,R} t_r^2}{2}
+
K_{\mathrm{ARB},R}
```

```math
K_\phi =
K_{\phi,F}
+
K_{\phi,R}
```

```math
\lambda_\phi =
\frac{K_{\phi,F}}
{K_{\phi,F}+K_{\phi,R}}
```

Elastic roll moment:

```math
M_{\phi,\mathrm{elastic}} =
m a_y
\left(
h_{\mathrm{CG}} - h_{\mathrm{RA}}
\right)
```

Elastic lateral load transfer:

```math
\Delta F_{z,\mathrm{lat,elastic},F} =
\frac{\lambda_\phi M_{\phi,\mathrm{elastic}}}{t_f}
```

```math
\Delta F_{z,\mathrm{lat,elastic},R} =
\frac{(1-\lambda_\phi) M_{\phi,\mathrm{elastic}}}{t_r}
```

Total lateral load transfer:

```math
\Delta F_{z,\mathrm{lat},F} =
\Delta F_{z,\mathrm{lat,geo},F}
+
\Delta F_{z,\mathrm{lat,elastic},F}
```

```math
\Delta F_{z,\mathrm{lat},R} =
\Delta F_{z,\mathrm{lat,geo},R}
+
\Delta F_{z,\mathrm{lat,elastic},R}
```

---

### 8.2 Dynamic suspension model

Planned as the most advanced simplified suspension model.

Main assumption:

```text
total load transfer = direct/geometric part + elastic delayed part
```

The direct/geometric part acts immediately.  
The elastic part is delayed with first-order dynamics.

Longitudinal total load transfer:

```math
\Delta F_{z,\mathrm{long,total}} =
\frac{m a_x h_{\mathrm{CG}}}{L}
```

Anti-squat / anti-dive direct fraction:

```math
\eta_{\mathrm{long}} =
\begin{cases}
AS, & a_x > 0 \quad \mathrm{drive/acceleration} \\
AD, & a_x < 0 \quad \mathrm{braking}
\end{cases}
```

Direct longitudinal part:

```math
\Delta F_{z,\mathrm{long,direct}} =
\eta_{\mathrm{long}}
\Delta F_{z,\mathrm{long,total}}
```

Elastic longitudinal target:

```math
\Delta F_{z,\mathrm{long,elastic,target}} =
(1-\eta_{\mathrm{long}})
\Delta F_{z,\mathrm{long,total}}
```

Elastic longitudinal dynamics:

```math
\dot{\Delta F}_{z,\mathrm{long,elastic}} =
\frac{
\Delta F_{z,\mathrm{long,elastic,target}}
-
\Delta F_{z,\mathrm{long,elastic}}
}
{\tau_{\mathrm{long}}}
```

Pitch stiffness:

```math
K_\theta =
2 k_{w,F} l_f^2
+
2 k_{w,R} l_r^2
```

Pitch natural frequency:

```math
f_\theta =
\frac{1}{2\pi}
\sqrt{
\frac{K_\theta}{I_\theta}
}
```

Longitudinal elastic time constant:

```math
\tau_{\mathrm{long}} =
\frac{1}
{2\pi \zeta_\theta f_\theta}
```

Roll axis height at the CG longitudinal position:

```math
h_{\mathrm{RA}} =
\frac{l_r}{L} h_{\mathrm{RC},F}
+
\frac{l_f}{L} h_{\mathrm{RC},R}
```

Global lateral direct/geometric and elastic fractions:

```math
\eta_{\mathrm{lat,geo}} =
\frac{h_{\mathrm{RA}}}{h_{\mathrm{CG}}}
```

```math
\eta_{\mathrm{lat,elastic}} =
\frac{h_{\mathrm{CG}}-h_{\mathrm{RA}}}{h_{\mathrm{CG}}}
```

Direct/geometric lateral load transfer:

```math
\Delta F_{z,\mathrm{lat,geo},F} =
\frac{F_{y,F} h_{\mathrm{RC},F}}{t_f}
```

```math
\Delta F_{z,\mathrm{lat,geo},R} =
\frac{F_{y,R} h_{\mathrm{RC},R}}{t_r}
```

Roll stiffness:

```math
K_{\phi,F} =
\frac{k_{w,F} t_f^2}{2}
+
K_{\mathrm{ARB},F}
```

```math
K_{\phi,R} =
\frac{k_{w,R} t_r^2}{2}
+
K_{\mathrm{ARB},R}
```

```math
K_\phi =
K_{\phi,F}
+
K_{\phi,R}
```

Front elastic roll moment distribution:

```math
\lambda_\phi =
\frac{K_{\phi,F}}
{K_{\phi,F}+K_{\phi,R}}
```

Elastic roll moment target:

```math
M_{\phi,\mathrm{elastic,target}} =
m a_y
\left(
h_{\mathrm{CG}} - h_{\mathrm{RA}}
\right)
```

Elastic lateral load transfer targets:

```math
\Delta F_{z,\mathrm{lat,elastic,target},F} =
\frac{
\lambda_\phi M_{\phi,\mathrm{elastic,target}}
}
{t_f}
```

```math
\Delta F_{z,\mathrm{lat,elastic,target},R} =
\frac{
(1-\lambda_\phi) M_{\phi,\mathrm{elastic,target}}
}
{t_r}
```

Elastic lateral dynamics:

```math
\dot{\Delta F}_{z,\mathrm{lat,elastic},F} =
\frac{
\Delta F_{z,\mathrm{lat,elastic,target},F}
-
\Delta F_{z,\mathrm{lat,elastic},F}
}
{\tau_{\mathrm{lat}}}
```

```math
\dot{\Delta F}_{z,\mathrm{lat,elastic},R} =
\frac{
\Delta F_{z,\mathrm{lat,elastic,target},R}
-
\Delta F_{z,\mathrm{lat,elastic},R}
}
{\tau_{\mathrm{lat}}}
```

Total lateral load transfer:

```math
\Delta F_{z,\mathrm{lat},F} =
\Delta F_{z,\mathrm{lat,geo},F}
+
\Delta F_{z,\mathrm{lat,elastic},F}
```

```math
\Delta F_{z,\mathrm{lat},R} =
\Delta F_{z,\mathrm{lat,geo},R}
+
\Delta F_{z,\mathrm{lat,elastic},R}
```

Roll natural frequency:

```math
f_\phi =
\frac{1}{2\pi}
\sqrt{
\frac{K_\phi}{I_\phi}
}
```

Lateral elastic time constant:

```math
\tau_{\mathrm{lat}} =
\frac{1}
{2\pi \zeta_\phi f_\phi}
```

Future version may replace the first-order elastic transfer with second-order roll/pitch dynamics:

```math
I_\phi \ddot{\phi}
+
C_\phi \dot{\phi}
+
K_\phi \phi
=
M_\phi
```

```math
I_\theta \ddot{\theta}
+
C_\theta \dot{\theta}
+
K_\theta \theta
=
M_\theta


```
## 9. GUI overview

Main panels:
- Top bar: load track, load/save config, reset defaults, select model, solve.
- Vehicle: mass, geometry, inertia, aero, rolling resistance.
- Tire: tire radius, nominal load, lateral tire parameters, longitudinal friction limit.
- Suspension: roll centers, wheel rates, ARBs, sprung masses, damping, anti-dive, anti-squat.
- Drivetrain: torque limits, power limits, drivetrain time constant.
- Solver: model, solve level, initial guess strategy, discretization, IPOPT settings.
- Bounds / Cost: optimization limits and weights.
- Files / Results: paths, export options, plots and summary.

---

## 10. Export

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

## 11. Not implemented yet

Not implemented yet:
- direct collocation integration,
- quasi-static suspension backend,
- dynamic suspension backend,
- complete initial guess vs NLP comparison tools.

---

## 12. Future work

Planned future extensions:
- add 4WD option,
- add hydraulic braking model,
- add accumulator energy usage model,
- add accumulator energy as state / decision variable,
- replace first-order elastic load transfer with second-order roll/pitch elastic transfer,
- improve solver diagnostics and sanity checks.
- add camber related effects
- add tire relaxation effects
- add heave related effects

---

## 13. References

- W. F. Milliken, D. L. Milliken, *Race Car Vehicle Dynamics*, SAE International, 1995.
  - https://books.google.com/books/about/Race_Car_Vehicle_Dynamics.html?id=opgHfQzlnLEC
- M. Massaro, D. J. N. Limebeer, *Minimum-lap-time optimisation and simulation*, Vehicle System Dynamics, 2021.
  - https://www.researchgate.net/publication/350662171_Minimum-lap-time_optimisation_and_simulation

---

## 14. Repository hygiene

Do not commit generated outputs or local dependencies.

Recommended ignored folders:

```text
external/
results/
```
