MATLAB App for Formula Student LTO-Based Vehicle Tuning
MATLAB App Designer project for simple Formula Student vehicle tuning based on LTO / minimum-lap-time optimization.
The app provides a GUI workflow for loading a track, editing vehicle and solver parameters, generating an initial guess, solving an NLP with CasADi + IPOPT, plotting results, and exporting solution/config/plots.
Current working backends:
```text
Point-mass free-line NLP
Dynamic bicycle NLP
```
Planned backend direction:
```text
Point mass -> dynamic bicycle -> quasi-static suspension -> dynamic suspension
```
---
1. CasADi installation
The NLP solvers use CasADi + IPOPT.
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
2. How to run
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
3. General workflow
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
For the dynamic bicycle backend, the automatic staged workflow is:
```text
backward-forward initial guess
        ↓
point-mass free-line NLP
        ↓
dynamic bicycle NLP
```
---
4. Track format
Expected CSV columns:
```text
x_left_m, y_left_m,
x_center_m, y_center_m,
x_right_m, y_right_m
```
All values are in meters.
The centerline is used as the reference path. The left and right lines define the available track boundaries.
---
5. Point-mass free-line NLP
The point-mass backend is a free-line NLP in Frenet coordinates.
State vector:
$$
x =
\begin{bmatrix}
e_y &
e_\psi &
v_x &
M_{\mathrm{cmd}} &
M_{\mathrm{rear}} &
t
\end{bmatrix}^{T}
$$
Control vector:
$$
u =
\begin{bmatrix}
\dot{M}{\mathrm{cmd}} &
\kappa{\mathrm{vehicle}}
\end{bmatrix}^{T}
$$
Here, $\kappa_{\mathrm{vehicle}}$ is a pseudo-curvature decision variable, not a real steering angle.
Progress speed:
$$
v_s =
\frac{v_x \cos(e_\psi)}
{1 - \kappa_{\mathrm{ref}} e_y}
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
Drag and rolling resistance:
$$
F_{\mathrm{drag}} = C_d v_x^2
$$
$$
F_{\mathrm{rr}} = C_r m g
$$
Drivetrain model:
$$
\dot{M}{\mathrm{rear}} =
\frac{M{\mathrm{cmd}} - M_{\mathrm{rear}}}{\tau}
$$
Lateral acceleration approximation:
$$
a_y =
v_x^2 \kappa_{\mathrm{vehicle}}
$$
Tire friction ellipse:
$$
\left(
\frac{F_{x,i}}{\mu_{x,i} F_{z,i}}
\right)^2
+
\left(
\frac{F_{y,i}}{\mu_{y,i} F_{z,i}}
\right)^2
\leq 1
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
J_{\dot{M}}
+
J_{\kappa}
J_v
$$
Torque-rate penalty:
$$
J_{\dot{M}} =
q_{\dot{M}}
\sum_k
\dot{M}_{\mathrm{cmd},k}^{2}
$$
Pseudo-curvature smoothness penalty:
$$
J_{\kappa} =
q_{\kappa}
\sum_k
\left(
\kappa_{\mathrm{vehicle},k+1}
\kappa_{\mathrm{vehicle},k}
\right)^2
$$
Optional progress-speed reward:
$$
J_v =
q_v
\sum_k
v_{s,k}
\frac{\Delta s_k}{L_{\mathrm{track}}}
$$
The main LTO objective is the final time `t_N`. The additional terms mainly regularize the solution.
---
6. Dynamic bicycle NLP
The dynamic bicycle backend is the first dynamic lateral model in the app.
State vector:
$$
x =
\begin{bmatrix}
e_y &
e_\psi &
v_x &
v_y &
r &
M_{\mathrm{cmd}} &
M_{\mathrm{rear}} &
\delta_{\mathrm{cmd}} &
\delta &
\dot{\delta} &
t
\end{bmatrix}^{T}
$$
Control vector:
$$
u =
\begin{bmatrix}
\dot{M}{\mathrm{cmd}} &
\dot{\delta}{\mathrm{cmd}}
\end{bmatrix}^{T}
$$
The model includes:
```text
lateral velocity v_y
yaw rate r
physical steering angle delta
second-order steering actuator model
first-order drivetrain model
rear-wheel-drive longitudinal force
axle-level tire friction ellipse
static normal loads with aerodynamic downforce
no dynamic mass transfer
```
Progress speed:
$$
v_s =
\frac{
v_x \cos(e_\psi)
v_y \sin(e_\psi)
}
{
1 - \kappa_{\mathrm{ref}} e_y
}
$$
Dynamic bicycle equations:
$$
\dot{v}x =
\frac{
F{x,R}
F_{yf}\sin(\delta)
F_{\mathrm{drag}}
F_{\mathrm{rr}}
}{m}
+
v_y r
$$
$$
\dot{v}y =
\frac{
F{yf}\cos(\delta)
+
F_{yr}
}{m}
v_x r
$$
$$
\dot{r} =
\frac{
l_f F_{yf}\cos(\delta)
l_r F_{yr}
}{I_z}
$$
Slip angles:
$$
\alpha_f =
\delta
\arctan
\left(
\frac{v_y + l_f r}{v_x}
\right)
$$
$$
\alpha_r =
\arctan
\left(
\frac{v_y - l_r r}{v_x}
\right)
$$
The lateral tire force is computed from an MF-like 5.2 tire model.
The axle-level friction ellipse is:
$$
\left(
\frac{F_{x,A}}{\mu_{x,A}F_{z,A}}
\right)^2
+
\left(
\frac{F_{y,A}}{\mu_{y,A}F_{z,A}}
\right)^2
\leq 1
$$
where `A` means front or rear axle.
Steering actuator model:
$$
\dot{\delta}_{\mathrm{cmd}}
u_{\delta}
$$
$$
\dot{\delta}
\dot{\delta}
$$
$$
\ddot{\delta}
\omega_n^2
\left(
\delta_{\mathrm{cmd}}
\delta
\right)
2\zeta\omega_n\dot{\delta}
$$
Objective function:
$$
J =
t_N
+
J_{\dot{M}}
+
J_{\dot{\delta}}
+
J_{\beta}
$$
Torque-rate penalty:
$$
J_{\dot{M}} =
q_{\dot{M}}
\sum_k
\dot{M}_{\mathrm{cmd},k}^{2}
$$
Steering command-rate penalty:
$$
J_{\dot{\delta}} =
q_{\dot{\delta}}
\sum_k
\dot{\delta}_{\mathrm{cmd},k}^{2}
$$
Beta consistency penalty:
$$
J_{\beta} =
q_{\beta}
\sum_k
\left(
\beta_{\mathrm{dyn},k}
\beta_{\mathrm{kin},k}
\right)^2
\Delta s_k
$$
where:
$$
\beta_{\mathrm{dyn}} =
\arctan
\left(
\frac{v_y}{v_x}
\right)
$$
$$
\beta_{\mathrm{kin}} =
\arctan
\left(
\frac{l_r}{L}
\tan(\delta)
\right)
$$
The beta cost does not directly penalize the value of beta. It penalizes the difference between dynamic beta and kinematic beta.
Main bounds used in the dynamic bicycle backend:
```text
v_x lower and upper bounds
M_cmd and M_rear torque bounds
M_cmd_dot torque command-rate bound
delta_cmd, delta and delta_dot steering bounds
delta_cmd_dot steering command-rate bound
track bounds on e_y
power limits
Frenet singularity protection
positive progress constraint
```
---
7. Backward-forward initial guess
The backward-forward solver is a fast initial-guess generator.
It estimates a speed profile from curvature and friction limits.
Basic relation:
$$
a_y =
v_x^2 \kappa
$$
Therefore:
$$
v_{x,\max} =
\sqrt{
\frac{a_{y,\max}}{|\kappa|}
}
$$
Then the algorithm applies forward acceleration and backward braking passes.
It is not the final optimizer. It only generates a useful seed for the NLP.
---
8. Planned model direction
8.1 Quasi-static suspension model
Planned as a model with immediate load transfer.
Main assumption:
```text
all load transfer is instantaneous / quasi-static / geometric
```
Longitudinal load transfer:
$$
\Delta F_{z,\mathrm{long}} =
\frac{m a_x h_{\mathrm{CG}}}{L}
$$
Roll stiffness distribution:
$$
K_{\phi,F} =
\frac{k_{w,F} t_f^2}{2}
+
K_{\mathrm{ARB},F}
$$
$$
K_{\phi,R} =
\frac{k_{w,R} t_r^2}{2}
+
K_{\mathrm{ARB},R}
$$
$$
\lambda_\phi =
\frac{K_{\phi,F}}
{K_{\phi,F}+K_{\phi,R}}
$$
8.2 Dynamic suspension model
Planned as the most advanced model.
Main assumption:
```text
total load transfer = geometric/direct part + elastic delayed part
```
The geometric part acts immediately. The elastic part is delayed with first-order dynamics:
$$
\dot{\Delta F}{z,\mathrm{elastic}} =
\frac{
\Delta F{z,\mathrm{elastic,target}}
\Delta F_{z,\mathrm{elastic}}
}{\tau}
$$
Natural frequency from wheel rate and sprung mass:
$$
f_n =
\frac{1}{2\pi}
\sqrt{
\frac{k_w}{m_s}
}
$$
First-order time constant from frequency and damping ratio:
$$
\tau =
\frac{1}{2\pi\zeta f_n}
$$
Future version may replace this first-order elastic transfer with second-order roll/pitch dynamics:
$$
I_\phi \ddot{\phi}
+
C_\phi \dot{\phi}
+
K_\phi \phi
M_\phi
$$
$$
I_\theta \ddot{\theta}
+
C_\theta \dot{\theta}
+
K_\theta \theta
M_\theta
$$
---
9. GUI overview
Main panels:
```text
Top bar: load track, load/save config, reset defaults, select model, solve
Vehicle: mass, geometry, inertia, aero, rolling resistance
Tire: tire radius, nominal load, lateral tire parameters, longitudinal friction limit
Suspension: roll centers, wheel rates, ARBs, sprung masses, damping, anti-dive, anti-squat
Drivetrain: torque limits, power limits, drivetrain time constant
Solver: model, solve level, initial guess strategy, discretization, IPOPT settings
Bounds / Cost: optimization limits and weights
Files / Results: paths, export options, plots and summary
```
---
10. Export
Available export options:
```text
Export NLP solution to MAT
Export NLP solution to CSV
Browse and export output package
```
The output package can include:
```text
NLP solution MAT
NLP CSV folder
config JSON
track CSV
initial guess MAT
backward-forward MAT
PNG plots
```
---
11. Not implemented yet
```text
direct collocation integration
quasi-static suspension backend
dynamic suspension backend
complete initial guess vs NLP comparison tools
```
---
12. Future work
```text
add 4WD option
add hydraulic braking model
add accumulator energy usage model
add accumulator energy as state / decision variable
replace first-order elastic load transfer with second-order roll/pitch elastic transfer
improve solver diagnostics and sanity checks
```
---
13. References
W. F. Milliken, D. L. Milliken, Race Car Vehicle Dynamics, SAE International, 1995.  
https://books.google.com/books/about/Race_Car_Vehicle_Dynamics.html?id=opgHfQzlnLEC
M. Massaro, D. J. N. Limebeer, Minimum-lap-time optimisation and simulation, Vehicle System Dynamics, 2021.  
https://www.researchgate.net/publication/350662171_Minimum-lap-time_optimisation_and_simulation
---
14. Repository hygiene
Do not commit generated outputs or local dependencies.
Recommended ignored folders:
```text
external/
results/
```