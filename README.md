# MATLAB App for Simple Formula Student Vehicle Tuning Based on LTO Results

This repository contains a MATLAB App Designer project for building and testing a Formula Student lap-time-optimization workflow.

The app is focused on GUI-based configuration, track loading, solver execution, result visualization, and export of LTO/NLP results. The general idea is to use several vehicle models of increasing complexity, starting from a simple point-mass NLP and later extending the app toward dynamic bicycle and suspension-aware models.

The optimization workflow is generally based on **nonlinear programming (NLP)** solved with **CasADi + IPOPT**.

At the current stage, the main implemented backend is:

```text
Point-mass free-line NLP
```

The other model modes are prepared in the GUI as future backends, but are not fully implemented yet.

---

## 1. Main purpose

The app is intended to help with:

- loading and previewing Formula Student tracks,
- configuring vehicle, tire, drivetrain, suspension, bounds, cost and solver parameters,
- generating a backward-forward initial guess,
- solving a point-mass free-line NLP,
- plotting solver results,
- comparing the final NLP solution with the initial guess,
- exporting results to MAT, CSV, JSON and PNG files.

The expected high-level workflow is:

```text
Track CSV
    +
GUI / JSON configuration
    ↓
Track preprocessing
    ↓
Initial guess generation
    ↓
CasADi + IPOPT NLP solve
    ↓
Common solution structure
    ↓
Plots and output export
```

---

## 2. How to run

### 2.1 Clone the repository

Using SSH:

```bash
git clone git@github.com:TymekProstak/Matlab-App-for-simple-Formula-Student-vehicle-tunning-based-on-LTO-results.git
cd Matlab-App-for-simple-Formula-Student-vehicle-tunning-based-on-LTO-results
```

Using HTTPS:

```bash
git clone https://github.com/TymekProstak/Matlab-App-for-simple-Formula-Student-vehicle-tunning-based-on-LTO-results.git
cd Matlab-App-for-simple-Formula-Student-vehicle-tunning-based-on-LTO-results
```

### 2.2 Start MATLAB in the project root

Open MATLAB and set the current folder to the repository root.

Then run:

```matlab
startup
```

This initializes the MATLAB path for the project.

### 2.3 Install CasADi

The NLP solver uses CasADi and IPOPT. If CasADi is not installed yet, run:

```matlab
install_casadi("auto")
```

The local dependency folder should be:

```text
external/
```

This folder is ignored by Git.

### 2.4 Launch the app

```matlab
LTO_tunner_app
```

---

## 3. Dependencies

The project currently uses:

- MATLAB,
- MATLAB App Designer,
- CasADi for MATLAB,
- IPOPT through CasADi,
- standard MATLAB plotting and export utilities.

Recommended ignored folders:

```text
external/
results/
```

`external/` is intended for local dependencies.  
`results/` is intended for generated outputs, plots and exported solver packages.

---

## 4. Track input format

The track is loaded from a CSV file.

Expected columns:

```text
x_left_m, y_left_m,
x_center_m, y_center_m,
x_right_m, y_right_m
```

The loaded track contains:

- left boundary,
- centerline,
- right boundary.

The solver preprocessing converts this into:

- arc length coordinate `s`,
- centerline heading,
- centerline curvature,
- left and right track width,
- total track width,
- interpolated solver grid.

For periodic tracks, the first and last track point should represent the same physical location or should be closed during preprocessing.

---

## 5. Current app workflow

A typical workflow is:

1. Load a track from CSV.
2. Load a configuration JSON or edit parameters manually in the GUI.
3. Select backend model and solve level.
4. Select initial guess strategy.
5. Click `Solve`.
6. The app:
   - reads the GUI configuration,
   - prepares the solver track,
   - optionally computes a backward-forward initial guess,
   - solves the NLP with CasADi + IPOPT,
   - stores the result in `app.LastSolution`.
7. Generate plots from:
   - NLP solution,
   - initial guess.
8. Export:
   - NLP solution to MAT,
   - NLP solution to CSV folder,
   - full output package.

---

## 6. Common solution format

The app uses a common `solution` structure. The goal is that every model eventually exports to the same general format.

Typical fields:

```matlab
solution.type
solution.status
solution.message

solution.s
solution.t
solution.lap_time

solution.X
solution.U
solution.state_names
solution.control_names

solution.global.x
solution.global.y
solution.global.psi

solution.track.s_ref
solution.track.kappa
solution.track.width_left
solution.track.width_right
solution.track.width_total

solution.wheel_names
solution.normal_loads

solution.tire_usage.x
solution.tire_usage.y
solution.tire_usage.total
```

For simpler models, some fields may be approximate placeholders. For example, the point-mass model does not have real lateral velocity dynamics, so `vy` is currently filled as zero.

---

## 7. Implemented model: backward-forward initial guess

The backward-forward solver is a fast heuristic used to create an initial speed profile.

It is not the final optimizer. It is used mainly to produce a better seed for the NLP.

The basic idea is:

```text
curvature + friction limits + acceleration/braking limits
    ↓
maximum feasible speed profile
    ↓
forward acceleration pass
    ↓
backward braking pass
    ↓
initial guess for NLP
```

The lateral acceleration relation is:

\[
a_y = v_x^2 \kappa
\]

so the curvature-based speed limit can be estimated as:

\[
v_{x,\max} =
\sqrt{
\frac{a_{y,\max}}{|\kappa|}
}
\]

The backward-forward result can be saved as an initial guess and compared with the final NLP solution.

---

## 8. Implemented model: point-mass free-line NLP

The currently implemented optimization backend is the point-mass free-line NLP.

It is formulated in Frenet coordinates and solved with CasADi + IPOPT.

### 8.1 States

The internal NLP state vector is:

\[
x =
\begin{bmatrix}
e_y \\
e_\psi \\
v_x \\
M_{\mathrm{cmd}} \\
M_{\mathrm{rear}} \\
t
\end{bmatrix}
\]

where:

- \(e_y\) is lateral deviation from the centerline,
- \(e_\psi\) is heading error relative to the centerline,
- \(v_x\) is longitudinal speed,
- \(M_{\mathrm{cmd}}\) is commanded rear torque,
- \(M_{\mathrm{rear}}\) is realized rear torque,
- \(t\) is reconstructed time.

### 8.2 Controls

The internal NLP control vector is:

\[
u =
\begin{bmatrix}
\dot{M}_{\mathrm{cmd}} \\
\kappa_{\mathrm{vehicle}}
\end{bmatrix}
\]

where:

- \(\dot{M}_{\mathrm{cmd}}\) is torque command rate,
- \(\kappa_{\mathrm{vehicle}}\) is a pseudo-curvature decision variable.

In this point-mass model, \(\kappa_{\mathrm{vehicle}}\) is not a physical steering angle. Steering-related fields are reconstructed after solving only to make the result useful as a seed for later dynamic models.

### 8.3 Frenet progress

The progress speed along the reference centerline is:

\[
v_s =
\frac{
v_x \cos(e_\psi)
}{
1 - \kappa_{\mathrm{ref}} e_y
}
\]

To avoid the Frenet singularity, the model imposes:

\[
1 - \kappa_{\mathrm{ref}} e_y \geq 0.2
\]

The model also enforces positive progress:

\[
v_s \geq v_{s,\min}
\]

### 8.4 Longitudinal force

Rear longitudinal force is computed directly from rear torque:

\[
F_x =
\frac{M_{\mathrm{rear}}}{R}
\]

The longitudinal acceleration is:

\[
a_x =
\frac{
F_x - F_{\mathrm{drag}} - F_{\mathrm{rr}}
}{m}
\]

with:

\[
F_{\mathrm{drag}} = C_d v_x^2
\]

\[
F_{\mathrm{rr}} = C_r m g
\]

### 8.5 Drivetrain dynamics

The realized rear torque follows the commanded torque using a first-order model:

\[
\dot{M}_{\mathrm{rear}} =
\frac{
M_{\mathrm{cmd}} - M_{\mathrm{rear}}
}{\tau}
\]

### 8.6 Lateral acceleration

The point-mass lateral acceleration is approximated as:

\[
a_y =
v_x^2 \kappa_{\mathrm{vehicle}}
\]

This is used for tire usage and for approximate steering reconstruction.

### 8.7 Normal loads

The point-mass model currently uses static load distribution plus aerodynamic downforce.

Front axle load:

\[
F_{z,F}
=
mg\frac{l_r}{L}
+
C_{L,F}v_x^2
\]

Rear axle load:

\[
F_{z,R}
=
mg\frac{l_f}{L}
+
C_{L,R}v_x^2
\]

Then each axle load is split equally left/right:

\[
F_{z,FL}=F_{z,FR}=\frac{1}{2}F_{z,F}
\]

\[
F_{z,RL}=F_{z,RR}=\frac{1}{2}F_{z,R}
\]

Dynamic mass transfer is not included in the point-mass model.

### 8.8 Tire friction ellipse

The point-mass NLP uses per-wheel tire usage constraints:

\[
\left(
\frac{F_{x,i}}{\mu_{x,i}F_{z,i}}
\right)^2
+
\left(
\frac{F_{y,i}}{\mu_{y,i}F_{z,i}}
\right)^2
\leq 1
\]

Longitudinal force is currently applied to the rear wheels:

\[
F_{x,FL}=F_{x,FR}=0
\]

\[
F_{x,RL}=F_{x,RR}=\frac{1}{2}F_x
\]

The lateral force is distributed between axles using a simple static axle relation:

\[
F_{y,F} = m a_y \frac{l_r}{L}
\]

\[
F_{y,R} = m a_y \frac{l_f}{L}
\]

and then split equally left/right.

### 8.9 Power constraint

Rear power is:

\[
P =
\frac{M_{\mathrm{rear}}v_x}{R}
\]

with constraints:

\[
P \leq P_{\mathrm{drive,max}}
\]

\[
P \geq -P_{\mathrm{brake,max}}
\]

### 8.10 Objective

The current objective has the form:

\[
J =
t_N
+
q_{\dot{M}}
\sum_k
\dot{M}_{\mathrm{cmd},k}^2
+
q_\kappa
\sum_k
\left(
\kappa_{\mathrm{vehicle},k+1}
-
\kappa_{\mathrm{vehicle},k}
\right)^2
-
q_v
\sum_k
v_{s,k}
\frac{\Delta s_k}{L_{\mathrm{track}}}
\]

The optimizer tries to minimize lap time, smooth torque command changes, smooth pseudo-curvature changes, and reward progress speed.

---

## 9. Planned model: dynamic bicycle

Status: not implemented yet.

The planned dynamic bicycle model is intended to be the first physically dynamic lateral model.

Main assumptions:

- no mass transfer,
- lateral dynamics included,
- yaw dynamics included,
- tire lateral force from either simplified MF / SMF or MF5.2-like lateral model,
- longitudinal force still obtained from torque,
- steering is a physical model variable rather than a pseudo-curvature control.

A typical state vector may contain:

\[
x =
\begin{bmatrix}
e_y \\
e_\psi \\
v_x \\
v_y \\
r \\
M_{\mathrm{cmd}} \\
M_{\mathrm{rear}} \\
\delta \\
\dot{\delta} \\
t
\end{bmatrix}
\]

where:

- \(v_y\) is lateral velocity,
- \(r\) is yaw rate,
- \(\delta\) is steering angle.

The lateral dynamics can be written in the general form:

\[
\dot{v}_y =
\frac{F_{yf}+F_{yr}}{m}
-
v_x r
\]

\[
\dot{r} =
\frac{l_fF_{yf}-l_rF_{yr}}{I_z}
\]

Slip angles:

\[
\alpha_f =
\delta -
\arctan
\left(
\frac{v_y+l_fr}{v_x}
\right)
\]

\[
\alpha_r =
-
\arctan
\left(
\frac{v_y-l_rr}{v_x}
\right)
\]

Lateral tire forces:

\[
F_{yf}=f_y(\alpha_f,F_{zf})
\]

\[
F_{yr}=f_y(\alpha_r,F_{zr})
\]

The tire function \(f_y\) is planned to be selectable between simplified MF/SMF and MF5.2-like forms.

This model should be more realistic than point mass, but still simpler than suspension-aware models because it does not include dynamic load transfer.

---

## 10. Planned model: quasi-static suspension

Status: not implemented yet.

The quasi-static suspension model is intended to include load transfer, but without dynamic suspension states.

Main assumption:

```text
All load transfer is treated as immediate / quasi-static.
```

So the model should compute wheel normal loads from:

- static axle loads,
- aerodynamic downforce,
- longitudinal load transfer,
- lateral load transfer,
- roll stiffness distribution,
- anti-dive and anti-squat geometry.

Longitudinal load transfer:

\[
\Delta F_{z,\mathrm{long}}
=
\frac{ma_xh_{\mathrm{CG}}}{L}
\]

Lateral load transfer can be split between front and rear axles using a roll stiffness distribution.

Front/rear roll stiffness:

\[
K_{\phi,F}
=
\frac{k_{w,F}t_f^2}{2}
+
K_{\mathrm{ARB},F}
\]

\[
K_{\phi,R}
=
\frac{k_{w,R}t_r^2}{2}
+
K_{\mathrm{ARB},R}
\]

Front elastic roll moment distribution:

\[
\lambda_\phi =
\frac{K_{\phi,F}}
{K_{\phi,F}+K_{\phi,R}}
\]

Then the lateral transfer can be distributed as:

\[
\Delta F_{z,\mathrm{lat},F}
=
\lambda_\phi
\frac{ma_yh_{\mathrm{eff}}}{t_f}
\]

\[
\Delta F_{z,\mathrm{lat},R}
=
(1-\lambda_\phi)
\frac{ma_yh_{\mathrm{eff}}}{t_r}
\]

In this quasi-static model, the transfer is immediate. There is no lag state for elastic transfer.

---

## 11. Planned model: dynamic suspension

Status: not implemented yet.

The dynamic suspension model is intended to be the most advanced model in the app.

The planned load-transfer concept is:

```text
total load transfer = geometric/direct part + elastic part
```

The geometric/direct part acts immediately.

The elastic part is delayed with first-order dynamics.

### 11.1 Lateral load transfer split

For lateral load transfer:

```text
geometric part:
    related to roll center / roll axis height

elastic part:
    related to suspension roll stiffness and roll compliance
```

A simple first-order elastic transfer state can be written as:

\[
\dot{\Delta F}_{z,\mathrm{elastic,lat}}
=
\frac{
\Delta F_{z,\mathrm{elastic,lat,target}}
-
\Delta F_{z,\mathrm{elastic,lat}}
}{
\tau_{\mathrm{lat}}
}
\]

### 11.2 Longitudinal load transfer split

For longitudinal load transfer:

```text
geometric/direct part:
    related to anti-dive / anti-squat

elastic part:
    related to pitch compliance
```

A first-order elastic longitudinal transfer state can be written as:

\[
\dot{\Delta F}_{z,\mathrm{elastic,long}}
=
\frac{
\Delta F_{z,\mathrm{elastic,long,target}}
-
\Delta F_{z,\mathrm{elastic,long}}
}{
\tau_{\mathrm{long}}
}
\]

### 11.3 Time constants from suspension frequencies

The first-order time constants should be derived from physically meaningful suspension data, not chosen arbitrarily.

A simple approximation is:

\[
\tau =
\frac{1}{\zeta \omega_n}
\]

where:

\[
\omega_n = 2\pi f_n
\]

so:

\[
\tau =
\frac{1}{2\pi \zeta f_n}
\]

The natural frequencies can be estimated from wheel rates and sprung masses:

\[
f_n =
\frac{1}{2\pi}
\sqrt{
\frac{k_w}{m_s}
}
\]

This allows the dynamic suspension model to compute approximate roll/pitch transfer time constants from GUI parameters such as wheel rates, sprung masses and damping ratios.

### 11.4 Future second-order version

The first-order elastic transfer model is planned as an intermediate version.

A later, more physical version should use second-order roll and pitch dynamics:

\[
I_\phi \ddot{\phi}
+
C_\phi \dot{\phi}
+
K_\phi \phi
=
M_\phi
\]

\[
I_\theta \ddot{\theta}
+
C_\theta \dot{\theta}
+
K_\theta \theta
=
M_\theta
\]

where:

- \(\phi\) is roll angle,
- \(\theta\) is pitch angle,
- \(I_\phi\), \(I_\theta\) are roll and pitch inertias,
- \(C_\phi\), \(C_\theta\) are damping coefficients,
- \(K_\phi\), \(K_\theta\) are roll and pitch stiffnesses,
- \(M_\phi\), \(M_\theta\) are roll and pitch excitation moments.

---

## 12. GUI panels

### 12.1 General top bar

The top bar contains the main workflow buttons:

- load track,
- load config,
- save config,
- reset to default,
- select backend model,
- select solve level,
- solve,
- status lamp and status label.

The backend model and solve level are also present in the Solver tab, so these dropdowns should stay synchronized.

### 12.2 Vehicle panel

Contains:

- mass,
- front and rear track width,
- wheelbase,
- CG-to-front distance,
- CG-to-rear distance,
- CG height,
- yaw inertia,
- roll inertia,
- pitch inertia,
- drag coefficient,
- front downforce coefficient,
- rear downforce coefficient,
- rolling resistance coefficient.

### 12.3 Tire panel

Contains:

- nominal load,
- tire radius,
- lateral relaxation length,
- simplified lateral tire parameters,
- MF5.2-like lateral parameters,
- longitudinal peak/friction-limit parameters.

The current point-mass model does not model longitudinal slip ratio. Longitudinal force comes from torque:

\[
F_x=\frac{M}{R}
\]

The longitudinal tire parameters are used to estimate longitudinal friction limit.

### 12.4 Suspension panel

Contains:

- front and rear roll center height,
- front and rear wheel rate,
- front and rear anti-roll bar stiffness,
- front and rear sprung mass,
- lateral/roll damping ratio,
- longitudinal/pitch damping ratio,
- anti-dive,
- anti-squat.

These parameters are mainly for the future quasi-static and dynamic suspension models.

### 12.5 Drivetrain panel

Contains:

- maximum drive torque,
- maximum brake/recuperation torque,
- maximum drive power,
- maximum brake/recuperation power,
- first-order drivetrain time constant.

### 12.6 Steering panel

Contains:

- maximum steering angle,
- steering natural frequency,
- steering damping ratio.

In the current point-mass NLP, steering constraints are not imposed directly. Steering fields are reconstructed after solving.

### 12.7 Solver panel

Contains:

- backend model,
- solve level,
- initial guess strategy,
- debug/preview/final number of points,
- integrator,
- integration substeps,
- IPOPT tolerances,
- maximum iterations,
- print level,
- acceptable tolerance.

### 12.8 Bounds panel

Contains:

- maximum beta / heading-error-like bound,
- maximum speed,
- minimum speed,
- minimum track margin,
- track width usage mode,
- maximum torque command rate,
- maximum steering command rate.

For the current point-mass NLP, steering-related bounds are not imposed directly.

### 12.9 Cost panel

Contains:

- speed reward,
- beta cost,
- torque rate cost,
- steering-rate / pseudo-curvature smoothness cost,
- slack costs.

In the current point-mass NLP, the steering-rate cost is used as a pseudo-curvature smoothness cost.

### 12.10 Files panel

Contains:

- track file path,
- config file path,
- initial guess path,
- output folder path,
- export mode,
- MAT export,
- CSV export,
- output package export.

### 12.11 Results panel

Contains:

- plot mode selector,
- plot type selector,
- generate selected plot button,
- plot area,
- summary table.

---

## 13. Export workflow

### 13.1 Export NLP solution to MAT

Exports `app.LastSolution` to a selected `.mat` file.

Saved variable:

```matlab
solution
```

### 13.2 Export NLP solution to CSV folder

Exports the NLP solution into readable CSV files, for example:

```text
nlp_nodes.csv
nlp_controls.csv
nlp_normal_loads.csv
nlp_tire_usage_x.csv
nlp_tire_usage_y.csv
nlp_tire_usage_total.csv
nlp_track_projection.csv
nlp_metadata.json
```

### 13.3 Browse and export output package

Creates a timestamped output folder and exports selected data depending on the export dropdown.

Possible exported files:

```text
nlp_solution.mat
config_current.json
track_original.csv
initial_guess.mat
backward_forward_solution.mat
```

Possible folders:

```text
nlp_csv/
plots_png/
```

Default plot names:

```text
trajectory_colored_by_speed.png
speed_profile.png
gg_plot.png
friction_usage_plot.png
controls_plot.png
command_rates_plot.png
torque_moment_plot.png
beta_angle_plot.png
slip_angles_plot.png
ey_epsi_plot.png
```

---

## 14. Not implemented yet

The following GUI/backend options are present or planned but not fully implemented yet:

- dynamic bicycle backend,
- direct collocation integration,
- quasi-static suspension backend,
- dynamic suspension backend,
- full periodic-track handling and validation,
- full comparison tools between initial guess and final NLP solution.

---

## 15. To do in future

Planned future work:

- implement dynamic bicycle NLP,
- implement direct collocation integration,
- implement quasi-static suspension model,
- implement dynamic suspension model,
- add 4WD drivetrain option,
- add hydraulic braking model,
- add battery energy consumption model,
- add battery energy as a state and/or decision variable,
- add wheel-level force allocation,
- add more complete tire model options,
- replace first-order elastic load transfer with second-order roll/pitch elastic transfer,
- improve solver diagnostics,
- improve periodic-track preprocessing,
- add automatic sanity-check plots after solve,
- add better export validation.

---

## 16. Repository hygiene

Do not commit generated results or local dependencies.

Recommended `.gitignore` entries:

```text
results/
external/
```

Use:

```text
external/    local dependencies such as CasADi
results/     generated outputs, plots, solver results
```