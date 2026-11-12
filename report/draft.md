---
title: "Automatic Scene Reset for Real-World Policy Evaluation on the DROID Platform"
author: [Tadeáš Jirák, Lenka Šimová]
date: "November 2026"
abstract: "TODO — write last."
---

<!--
Drafting conventions (pandoc + pandoc-crossref, converted to the TeX template at the end):
- Math in $…$ / $$…$$ {#eq:label}; figures ![caption](figures/x.pdf){#fig:label}; tables {#tbl:label}; cites [@key].
- All numbers come from report/facts.md (or lab data). Notation is fixed in facts.md §1.
- "TODO(lab)" = needs a measurement/photo from the lab; "TODO(ask)" = question for Tadeáš.
Status: §2 and §3 drafted; §1, §5–7 and the abstract are written last.
-->

# Introduction

TODO — write after the results. Contents: see outline.md.

# Problem Analysis and Specification {#sec:analysis}

## Task formulation {#sec:task}

Our station follows the hardware setup of DROID [@khazatsky2024droid]: a
Franka Emika Panda 7-DoF arm with a Robotiq two-finger gripper and
a stereo camera mounted on the wrist, controlled through the DROID robot
server. Next to the robot lies a _grid_: a 6 × 6 arrangement of Gridfinity cells with a cell
pitch of $a = 42$ mm, on which objects are stored in defined cells. Three cells
of the grid hold fixed tiles carrying ArUco markers, which define the grid
coordinate frame $g$. The manipulated objects are 40 × 40 × 20 mm cubes with one
ArUco marker on the top face.

A reset request specifies a _target configuration_: for every listed
object $o$, the grid cell $(i,j)$ in which it should end up. The cell determines
the desired object pose $\mathbf{T}^{\star}_{go}$ in the grid frame
(§@sec:targets). The current object poses are unknown at the time of the
request. To perform the reset, the system has to

1. localize the grid and the objects, and
2. plan and execute one pick-and-place transfer per object that brings the
   object to its target pose.

Every reset produces a log of all observations, planned endpoints, commands
and failures, so that each reset can be audited later.

**Placement quality.** We judge a reset by the pose $\mathbf{T}_{bo}$ in which
each object ends up, measured after the reset and expressed in the robot
base frame. We compare it with the requested pose in the same frame,
$\mathbf{T}^{\star}_{bo} = \mathbf{T}_{bg}\,\mathbf{T}^{\star}_{go}$:

$$
e_p = \bigl\| \mathbf{t}_{bo} - \mathbf{t}^{\star}_{bo} \bigr\|,
\qquad
e_R = \bigl\| \log\!\bigl(\mathbf{R}^{\top}_{bo}\,\mathbf{R}^{\star}_{bo}\bigr)^{\vee} \bigr\|,
$$ {#eq:errors}
where $e_p$ is the position error and $e_R$ the angle of the relative
rotation. A reset is *successful* if every target object satisfies
$e_p \le \varepsilon_p$ and $e_R \le \varepsilon_R$, with $\varepsilon_p = 10$ mm and
$\varepsilon_R = 0.1$ rad (5.7°). TODO: one sentence on why these tolerances (a
cube is 40 mm wide and a cell 42 mm wide, so an error of 10 mm is still clearly
"in the cell"). The final pose is measured independently of the transfer: in
our experiments by the wrist camera and by hand (§@sec:experiments). The
system accepts such a measurement and computes ([@eq:errors]) and the success
flag (§@sec:verification).

## Decomposition {#sec:decomposition}

We split the task into five sub-problems connected by explicitly defined
interfaces (Fig. @fig:pipeline):

| Sub-problem | Input | Output |
|---|---|---|
| Grid localization | stereo image pair, flange pose $\mathbf{T}_{bf}$ | $\mathbf{T}_{bg}$ (cached) |
| Object localization | stereo image pair, $\mathbf{T}_{bf}$ | $\mathbf{T}_{bo}$ for each visible object |
| Transfer planning | $\mathbf{T}_{bo}$, $\mathbf{T}_{bg}$, $\mathbf{T}^{\star}_{go}$, current flange pose | ordered list of flange endpoints and gripper actions |
| Execution | endpoints | joint targets sent to the robot; executed motion |
| Placement evaluation | measured final $\mathbf{T}_{bo}$, targets | $e_p$, $e_R$, success flag |

: Sub-problems of the scene reset and their interfaces. {#tbl:interfaces}

![Pipeline of one reset. TODO: block diagram (TikZ).](figures/pipeline.pdf){#fig:pipeline}

All poses passed between the components are rigid transforms in metres, and
every pose carries the name of its reference frame. Observations are
expressed in the robot base frame $b$ and targets in the grid frame $g$. Each
interface is therefore unambiguous, even though different components work in
different frames.

## Design decisions {#sec:design}

**Fiducial markers instead of learned pose estimation.** Both the grid and the
objects carry ArUco markers [@garrido2014aruco]. A marker of known size
yields a metric 6-DoF pose from a single image and a sub-pixel corner residual
that can be inspected. Learned pose estimators avoid the markers,
but their errors are harder to bound. Since the purpose of the station is to
produce *trustworthy* resets, we accept the restriction to marked objects.

**Static grid, localized once.** The grid is assumed not to move during an
evaluation session. Its pose is estimated from one stationary stereo capture and
cached together with the image it was computed from. Later resets reuse the
cached pose without looking at the grid again.

**Two-stage object observation.** The first observation of a cube comes from
wherever the arm currently is, often far away and at an oblique angle. Before
grasping, the arm therefore moves the camera to a close-up pose
directly above the marker and localizes the cube again. The
transfer is then planned from this refined pose.

**Endpoints and inverse kinematics instead of full motion planning.** A transfer
is a short fixed sequence of flange endpoints at a safe transit height. Each
endpoint is converted to a joint target by inverse kinematics (IK), and the
robot moves to it in joint space. This keeps the behaviour simple and
predictable, but it requires a free corridor above the workspace (see
§@sec:scope).

**Success only from measurement.** A completed motion is not proof of a
correct placement: the grasp may slip and controllers may report success
incorrectly. The system therefore never derives success from the executed
commands; only a measured final pose can establish it.


**No automatic retries.** A failed or uncertain command aborts the reset and is
logged. Retrying a motion on a real robot without knowing why it failed can
make the situation worse.

## Related work {#sec:related}

TODO (½ page). Planned content, citations to verify:

- DROID platform and dataset [@khazatsky2024droid].
- Automating real-world policy evaluation, including learned reset policies:
  AutoEval [@zhou2025autoeval]; distributed real-world evaluation, RoboArena
  [@atreya2025roboarena]. Simulation as a proxy and its limits, SIMPLER
  [@li2024simpler].
- Scene resetting in reset-free / autonomous RL (e.g. [@gupta2021resetfree]).
- Pose from square fiducials: ArUco [@garrido2014aruco], IPPE
  [@collins2014ippe]; Levenberg–Marquardt [@more1978lm].
- GPU-parallel IK and motion generation: cuRobo [@sundaralingam2023curobo].
- Gridfinity modular storage standard (web reference).

Positioning sentence: unlike learned reset policies, we use an explicit,
geometric pipeline whose every step can be measured and logged. The price is
that the objects must carry markers.

## Assumptions and scope {#sec:scope}

The system is designed for the following setting:

- **Objects.** Upright marker-topped cubes, grasped from above with one
  configured grasp.
- **Workspace.** The grid stays in place during a session, and the space
  above the table and the grid is free of obstacles. Inverse kinematics checks
  the arm's self-collision; the environment is not modelled.
- **Interfaces.** The reset is requested by an external caller (e.g. an
  evaluation station between policy rollouts). The final scene measurement used
  for placement evaluation is supplied to the system.

Running policies, scoring rollouts and orchestrating evaluations are outside
the scope of this work.

# Method {#sec:method}

## Frames and notation {#sec:frames}

We write $\mathbf{T}_{BA} \in SE(3)$ for the transformation from frame $A$ to
frame $B$. It maps a point with coordinates $\mathbf{x}_A$ in $A$ to its
coordinates $\mathbf{x}_B = \mathbf{T}_{BA}\,\mathbf{x}_A$ in $B$ (homogeneous
coordinates). Equivalently, it is the pose of frame $A$ expressed in frame $B$.
$\mathbf{R}_{BA}$ and $\mathbf{t}_{BA}$ denote its rotation and translation,
$\mathbf{T}_{AB} = \mathbf{T}_{BA}^{-1}$, and transformations compose with
matching inner indices, $\mathbf{T}_{CA} = \mathbf{T}_{CB}\,\mathbf{T}_{BA}$.
The frames used throughout the report are (Fig. @fig:frames):

- $b$, the robot base;
- $f$, the robot flange (link `panda_link8`, the frame reported by DROID);[^tool]
- $c$ and $r$, the left and right optical frames of the rectified stereo
  camera ($x$ right, $y$ down, $z$ along the optical axis);
- $g$, the grid: origin at the outer corner of cell $(0,0)$, $x$ along
  increasing columns, $y$ along increasing rows, $z$ out of the marker faces,
  with $z = 0$ on the plane of the marker tops;
- $o$, the cube (object): origin at its geometric centre, axes along its edges, $z$
  through the top face;
- $m$, the cube marker: origin at the marker centre, $z$ out of the face.

[^tool]: The code calls this frame the *tool* frame.

The camera is rigidly mounted on the wrist. Its mounting transform
$\mathbf{T}_{fc}$ was calibrated beforehand (TODO(ask): by whom and with which
method). The rectified stereo geometry $\mathbf{T}_{cr}$ is provided by the
camera SDK. Since the images are rectified, the projection
$\pi(\mathbf{K}_e, \mathbf{X})$ of a 3D point $\mathbf{X}$ into eye $e \in \{c, r\}$
is an ideal pinhole projection with intrinsic matrix $\mathbf{K}_e$ and no lens
distortion.

![Coordinate frames and the transform chain from the robot base to the grid
and to the cube. TODO: diagram.](figures/frames.pdf){#fig:frames}

Rotations given as three angles use the fixed-axis XYZ convention,
$\mathbf{R} = \mathbf{R}_z(\gamma)\mathbf{R}_y(\beta)\mathbf{R}_x(\alpha)$ for
(roll, pitch, yaw) $= (\alpha, \beta, \gamma)$.

## Grid localization {#sec:grid}

**Model.** Marker $j$ lies in a known cell $(i_j, k_j)$ whose centre is
$$
\mathbf{c}_j = \bigl( (i_j + \tfrac12)\,a,\; (k_j + \tfrac12)\,a,\; 0 \bigr)^{\top}.
$$
{#eq:cell}

The markers are 3D-printed into Gridfinity tiles. Each tile can sit in its
cell in any of four orientations, and the print need not be exactly square to
the cell. We therefore do not assume the in-plane rotation of a marker but
estimate it as an extra unknown $\phi_j$. With $h = s_m/2$ the half-side of a
grid marker ($s_m = 32$ mm) and the four corners of a centred square


$$

\mathbf{s}_1 = (-h, h, 0)^{\top},\;
\mathbf{s}_2 = (h, h, 0)^{\top},\;
\mathbf{s}_3 = (h, -h, 0)^{\top},\;
\mathbf{s}_4 = (-h, -h, 0)^{\top},

$$
{#eq:corners}


in the order in which the ArUco detector returns them, corner $k$ of marker $j$
has the grid coordinates
$\mathbf{X}_{jk} = \mathbf{c}_j + \mathbf{R}_z(\phi_j)\,\mathbf{s}_k$.

**Joint stereo fit.** The unknowns are the grid pose in the left camera,
$\mathbf{T}_{cg}$, parametrized by a rotation vector and a translation,
and the marker yaws $\phi_j$. Let $\mathbf{u}_{ejk}$ be the detected
image position of corner $k$ of marker $j$ in eye $e$, and let
$\mathbf{T}_{cc} = \mathbf{I}$, $\mathbf{T}_{rc} = (\mathbf{T}_{cr})^{-1}$.
We minimize the reprojection error over all corners that were actually detected
in either eye:


$$
\min_{\mathbf{T}_{cg},\,\{\phi_j\}}
\sum_{(e,j,k)\,\in\,\mathcal{D}}
\bigl\| \pi\bigl(\mathbf{K}_e,\; \mathbf{T}_{ec}\,\mathbf{T}_{cg}\,\mathbf{X}_{jk}(\phi_j)\bigr) - \mathbf{u}_{ejk} \bigr\|^2 .
$$
{#eq:grid-fit}

The yaws are shared between the eyes. Because each $\phi_j$ is free, one marker
alone does not determine the rotation of the grid about its normal, so
at least two distinct markers must be detected (in total over both eyes). An
eye that sees zero or one marker still contributes its corners.

**Initialization.** Problem ([@eq:grid-fit]) is non-convex, so it is
started from closed-form guesses. For every visible marker, the IPPE method for
squares [@collins2014ippe] returns up to two candidate poses
$(\mathbf{R}_j, \mathbf{t}_j)$ in the camera frame; the two-fold ambiguity
is typical for small planar targets. For every pair of markers $(i,j)$ and every
combination of their candidates we build a grid pose:

1. the grid normal $\mathbf{n}$ is the normalized sum of the two marker
   $z$-axes;
2. the in-plane direction $\mathbf{d}$ is $\mathbf{t}_j - \mathbf{t}_i$ with its
   component along $\mathbf{n}$ removed and normalized; the same direction in
   the grid frame is $\mathbf{d}_g = (\mathbf{c}_j - \mathbf{c}_i)/\|\mathbf{c}_j - \mathbf{c}_i\|$;
3. the rotation maps the grid triad $(\mathbf{d}_g, \mathbf{e}_z \times \mathbf{d}_g, \mathbf{e}_z)$
   onto $(\mathbf{d}, \mathbf{n} \times \mathbf{d}, \mathbf{n})$:


$$
\mathbf{R}\_{cg} = \mathbf{d}\,\mathbf{d}\_g^{\top}

- (\mathbf{n} \times \mathbf{d})(\mathbf{e}\_z \times \mathbf{d}\_g)^{\top}
- \mathbf{n}\,\mathbf{e}\_z^{\top};
  $$ {#eq:init-rotation}

4. the translation is the average of $\mathbf{t}_i - \mathbf{R}_{cg}\mathbf{c}_i$
   and $\mathbf{t}_j - \mathbf{R}_{cg}\mathbf{c}_j$;
5. each yaw $\phi_j$ is read from the candidate whose normal agrees best with
   $\mathbf{n}$: with $\mathbf{Q} = \mathbf{R}^{\top}_{cg}\mathbf{R}_j$,
   $\phi_j = \operatorname{atan2}(Q_{21} - Q_{12},\, Q_{11} + Q_{22})$.

The eight initial guesses with the lowest cost are refined with a
trust-region least-squares solver (SciPy `least_squares`, method `trf`, at
most 120 function evaluations, tolerances $10^{-9}$). A solution is accepted only if
every fitted corner lies in front of the camera that observed it and the grid
faces that camera. Of the accepted solutions, the one with the lowest root-mean-square (RMS)
reprojection error is returned. If the joint fit fails, each eye is fitted
separately and the two estimates are fused (Appendix TODO; this fallback is a
heuristic).

**Base frame.** The robot holds still during the capture, and the flange pose
$\mathbf{T}_{bf}$ is read from the controller. The grid pose in the base
frame is

$$
\mathbf{T}_{bg} = \mathbf{T}_{bf}\;\mathbf{T}_{fc}\;\mathbf{T}_{cg}.
$$ {#eq:base-grid}
The result is stored together with the source image, the fit residuals and the
configuration it was computed for (marker layout, camera, mounting transform).
A stored pose is reused only if all of these still match and the RMS error
is at most 2 px or an operator accepted the capture.

## Cube localization {#sec:cube}

Each cube carries a unique ArUco marker, so detecting the marker also
identifies the cube. The marker pose in the left camera,
$\mathbf{T}_{cm}$, is estimated with the same approach as the grid, but
with only six unknowns. The corners are ([@eq:corners]) with
$h = s_c/2$, where $s_c$ is the side of the cube marker. We minimize
$$
\min_{\mathbf{T}_{cm}}
\sum_{e}\sum_{k=1}^{4}
\bigl\| \pi\bigl(\mathbf{K}_e,\; \mathbf{T}_{ec}\,\mathbf{T}_{cm}\,\mathbf{s}_k\bigr) - \mathbf{u}_{ek} \bigr\|^2
$$
{#eq:cube-fit}
over the eyes in which the marker was detected, so one eye is
sufficient. The IPPE candidates of each eye are transformed into the left
frame. The candidate with the lowest cost in ([@eq:cube-fit]) is the starting
point, and the problem is solved by the Levenberg–Marquardt method
[@more1978lm]. A fit that places any corner behind a camera is rejected. No
threshold on the reprojection error is applied; the RMS error is logged with
each observation instead.

The cube pose follows from the known placement of the marker on the cube:
$$
\mathbf{T}_{bo} = \mathbf{T}_{bf}\;\mathbf{T}_{fc}\;\mathbf{T}_{cm}\;\bigl(\mathbf{T}_{om}\bigr)^{-1},
\qquad
\mathbf{T}_{om} =
\begin{bmatrix} \mathbf{R}\_z(\psi_m) & (0,\,0,\,h_o/2)^{\top} \\ \mathbf{0}^{\top} & 1 \end{bmatrix},
$$
{#eq:cube-pose}
where $h_o = 20$ mm is the cube height and $\psi_m = 0$ is the marker's yaw
relative to the cube edges. The marker is centred on the top face.

The side length $s_c$ directly sets the scale of the estimated translation. If
the true side is $s_c'$, the estimated camera-to-marker distance is wrong by the
factor $s_c/s_c'$. TODO(lab): our configuration uses $s_c = 37$ mm, marked in
the repository as an assumed value. Measure it and report the measured value.

## Two-stage observation {#sec:scan}

The first estimate of $\mathbf{T}_{bo}$ is obtained from the robot's
current pose and may be imprecise (a distant, oblique view). We therefore
move the camera to a close-up pose above the marker and observe the cube again.
The scan pose is chosen so that

- the optical axes point straight down ($-z$ of the base frame);
- the camera keeps its current heading, given by the yaw
  $\psi_c = \operatorname{atan2}(R_{21}, R_{11})$ of $\mathbf{R}_{bc}$, so that the wrist does
  not rotate unnecessarily;
- the marker lies on the axis through the midpoint of the two cameras, at a
  distance $D = 0.25$ m.

With $\mathbf{q} = \tfrac12\,\mathbf{t}_{cr}$ the midpoint between the
two optical centres (expressed in the left frame) and $\mathbf{t}_{bm}$ the
current estimate of the marker centre, this gives
$$
\mathbf{R}^{\mathrm{scan}}_{bc} =
\begin{bmatrix}
\cos\psi_c & \sin\psi_c & 0\\
\sin\psi_c & -\cos\psi_c & 0\\
0 & 0 & -1
\end{bmatrix},
\qquad
\mathbf{t}^{\mathrm{scan}}_{bc} = \mathbf{t}_{bm} - \mathbf{R}^{\mathrm{scan}}_{bc}\,\bigl(\mathbf{q} + D\,\mathbf{e}_z\bigr),
$$
{#eq:scan-pose}
and the flange target
$\mathbf{T}^{\mathrm{scan}}_{bf} = \mathbf{T}^{\mathrm{scan}}_{bc}\,(\mathbf{T}_{fc})^{-1}$.
The scan stage consists of four motions and one observation: lift to the
transit height, open the gripper, move above the scan pose, descend to it, and
localize the cube. If the close-up observation fails, the reset continues
with the first estimate and logs a warning. The objects are processed one at a time: scan, then transfer, then the next
object.

## Target poses {#sec:targets}

A cube placed in cell $(i,j)$ stands on the cell floor, centred in the cell.
Its target pose in the grid frame is therefore
$$
\mathbf{T}^{\star}_{go} =
\begin{bmatrix}
\mathbf{R}_z(\psi) & \bigl((i+\tfrac12)a,\; (j+\tfrac12)a,\; z_s + h_o/2\bigr)^{\top}\\
\mathbf{0}^{\top} & 1
\end{bmatrix},
$$
{#eq:target}
where $z_s$ is the height of the cell floor relative to the marker plane and
$h_o/2$ lifts the cube centre above the floor. The floor height is a property of
the physical grid. We measure it once (TODO(lab): value) rather than assume that
the floor coincides with the marker plane.

The yaw $\psi$ is the only free choice beyond the cell, and by default
$\psi = 0$. In practice the cell leaves almost no freedom. A cube of
footprint $w \times l$ fits into the cell only if
$|\cos\psi|\,w + |\sin\psi|\,l \le a$. For our 40 mm cube in a 42 mm cell,
this allows $|\psi| \le 2.9^{\circ}$ around $0^{\circ}$, $90^{\circ}$,
$180^{\circ}$ or $270^{\circ}$. The yaw therefore selects which way the
cube's marker faces. Targets in cells occupied by a marker tile, or with a
footprint that does not fit, are rejected.

## Transfer planning {#sec:planning}

**Grasp.** The cube is grasped from above, with the flange $z$-axis pointing
down and the grasp centre at the cube centre:
$$
\mathbf{T}_{of} =
\begin{bmatrix}
\mathbf{R}_z(\psi_g)\,\mathbf{R}_x(\pi) & (0,\,0,\,d_g)^{\top}\\
\mathbf{0}^{\top} & 1
\end{bmatrix},
$$
{#eq:grasp}
where $d_g = 0.167$ m is the distance from the flange to the grasp centre
between the closed fingers and $\psi_g = 0$ is the grasp yaw relative to the cube.
$\mathbf{R}_x(\pi)$ turns the flange upside down, so that its $z$-axis, which points along the
gripper, points down into the cube. $\mathbf{R}_z(\psi_g)$ selects which pair of
opposite faces the fingers close on. The same grasp transform is used for picking and placing, so the
pick and place flange poses are
$$
\mathbf{T}^{\mathrm{pick}}_{bf} = \mathbf{T}_{bo}\,\mathbf{T}_{of},
\qquad
\mathbf{T}^{\mathrm{place}}_{bf} = \mathbf{T}_{bg}\,\mathbf{T}^{\star}_{go}\,\mathbf{T}_{of}.
$$
{#eq:pick-place}

**Endpoint sequence.** Motions between the pick and the place happen at a common
transit height
$$
z_h = \max\bigl(z_T,\; z_{\mathrm{cur}},\; z_{\mathrm{pick}} + \Delta,\; z_{\mathrm{place}} + \Delta\bigr),
$$
{#eq:transit}
where $z_T = 0.40$ m is the configured transit height in the base frame,
$z_{\mathrm{cur}}$, $z_{\mathrm{pick}}$ and $z_{\mathrm{place}}$ are the flange
heights of the current, pick and place poses, and $\Delta = 0.08$ m is the
minimum lift clearance. Let $\mathrm{above}(\mathbf{T})$ denote the pose
$\mathbf{T}$ with its $z$-coordinate replaced by $z_h$. A transfer consists of
ten actions (Fig. @fig:sequence):

| # | Action | Flange target |
|---|---|---|
| 1 | initial lift | $\mathrm{above}(\mathbf{T}^{\mathrm{cur}}_{bf})$ |
| 2 | open gripper | — |
| 3 | above pick | $\mathrm{above}(\mathbf{T}^{\mathrm{pick}}_{bf})$ |
| 4 | pick | $\mathbf{T}^{\mathrm{pick}}_{bf}$ |
| 5 | close gripper | — |
| 6 | lift | $\mathrm{above}(\mathbf{T}^{\mathrm{pick}}_{bf})$ |
| 7 | above place | $\mathrm{above}(\mathbf{T}^{\mathrm{place}}_{bf})$ |
| 8 | place | $\mathbf{T}^{\mathrm{place}}_{bf}$ |
| 9 | open gripper | — |
| 10 | retreat | $\mathrm{above}(\mathbf{T}^{\mathrm{place}}_{bf})$ |

: The ten actions of one transfer. {#tbl:actions}

![Side view of the endpoint sequence of one transfer. TODO: diagram.](figures/sequence.pdf){#fig:sequence}

**Fingertip floor.** Before IK, every flange target is checked so that the closed
fingertips, at $d_{\mathrm{tip}} = 0.178$ m along the flange $z$-axis, stay at
least 2 mm above the table (table surface at $z = -0.040$ m in the base frame,
limit $z_{\min} = -0.038$ m). With $R_{33}$ the $z$-component of the flange
$z$-axis, the fingertip height is $z_f + R_{33}\,d_{\mathrm{tip}}$. If it is
below $z_{\min}$, the target is raised to
$$
z_f \leftarrow z_{\min} - R_{33}\,d_{\mathrm{tip}},
$$
{#eq:clip}
keeping $x$, $y$ and the orientation.



## Inverse kinematics and execution {#sec:execution}

**IK.** All flange endpoints of a stage (the scan stage or the transfer) are
converted to joint targets *before* the robot moves. If any endpoint has no
solution, nothing is executed. We use the GPU-parallel IK solver of cuRobo
[@sundaralingam2023curobo] with 32 random seeds, a position tolerance of 1 mm
and an orientation tolerance of 0.01 rad. The joint limits are those of the
DROID hardware configuration, shrunk by a 0.2 rad margin. Self-collision of the
arm links is checked; the hand geometry of the standard Panda model is removed,
because the robot carries a different gripper. Among the valid solutions we
choose the one closest to the previous joint configuration,
$$
\mathbf{q}_n = \arg\min_{\mathbf{q}\,\in\,\mathcal{Q}_n} \|\mathbf{q} - \mathbf{q}_{n-1}\|_2^2 ,
$$
{#eq:ik-select}
where $\mathcal{Q}_n$ is the set of valid IK solutions for endpoint $n$ and
$\mathbf{q}_0$ is the measured joint configuration. Chaining the endpoints this way
avoids large reconfigurations of the arm between consecutive endpoints.

**Execution.** Each joint target is sent to the DROID server as one blocking
command. The next action starts only after the arm has come to rest:
all joint speeds must stay below 0.02 rad/s for 0.2 s (checked every
50 ms, at most 60 s). The server must also report that the previous command
succeeded. Gripper commands (fully open or fully closed) are followed by a fixed
2.5 s wait. The executed pose is not compared with the commanded endpoint. Any
error, timeout or unsuccessful command stops the reset, which is then marked
*failed*. An operator can cancel the reset between two commands.

Between two endpoints, the robot controller moves every joint smoothly from its
current to its target angle. Such a path is smooth in joint space but generally
curved for the gripper. The vertical descents and ascents in
Table @tbl:actions (actions 3→4, 4→6, 7→8 and 8→10) are therefore vertical only
at their endpoints; in between, the gripper may deviate sideways. For these short
motions the deviation is small, because consecutive joint configurations are
close ([@eq:ik-select]), but it is not controlled. This is one reason the
workspace above the table must be clear.

## Placement evaluation {#sec:verification}

After the last action, the system keeps the reset open for evaluation (state
*awaiting verification*), with the outcome still unknown. When a final pose
measurement is supplied for every target object, it is compared with the target
using ([@eq:errors]). The errors $e_p$ and $e_R$, the success flag and the final
scene are stored with the reset. An object without a final measurement counts
as a failure. The
rotation error compares full 3D orientations. The cube's square
symmetry is not used to reduce the error, because the marker makes the four
yaw orientations distinguishable.

# Implementation {#sec:implementation}

TODO — brief. Bullets to expand:

- Hardware: Panda + DROID, Robotiq gripper (TODO(lab) model), ZED wrist camera
  (TODO(lab) model) at 1280 × 720 px per eye and 15 FPS, control PC with an NVIDIA
  GPU (TODO(lab)), DROID ZeroRPC server on a NUC. Fig. @fig:station (photo).
- Software: Python; OpenCV ArUco (sub-pixel corner refinement), SciPy,
  cuRobo + PyTorch (CUDA), ZED SDK. Three packages: `scene` (physical objects
  and grasp geometry), `buffer` (geometry and localization of the grid),
  `auto_reset` (planning, execution, verification, I/O).
- Interface: JSON requests (`observe`, `plan`, `reset`, `verify`, `state`),
  one per line; the manager keeps state between requests.
- Logging: JSONL events; images and fit diagnostics of every scan.
- Camera exposure: custom highlight-protecting exposure control.
- Safety: motion only with an explicit flag; no retries; endpoint height floor.
- Parameter table → Appendix A.

# Experiments {#sec:experiments}

TODO — after the lab session. See the plan in outline.md.

# Discussion {#sec:discussion}

TODO.

# Conclusion {#sec:conclusion}

TODO.

# References

<!-- To verify and convert to BibTeX:
khazatsky2024droid — Khazatsky et al., DROID: A Large-Scale In-the-Wild Robot Manipulation Dataset, RSS 2024.
zhou2025autoeval — Zhou et al., AutoEval: Autonomous Evaluation of Generalist Robot Manipulation Policies in the Real World, 2025.
atreya2025roboarena — Atreya et al., RoboArena: Distributed Real-World Evaluation of Generalist Robot Policies, 2025.
li2024simpler — Li et al., Evaluating Real-World Robot Manipulation Policies in Simulation (SIMPLER), CoRL 2024.
gupta2021resetfree — Gupta et al., Reset-Free Reinforcement Learning via Multi-Task Learning, ICRA 2021.
garrido2014aruco — Garrido-Jurado et al., Automatic generation and detection of highly reliable fiducial markers under occlusion, Pattern Recognition 2014.
collins2014ippe — Collins & Bartoli, Infinitesimal Plane-Based Pose Estimation, IJCV 2014.
more1978lm — Moré, The Levenberg–Marquardt algorithm: implementation and theory, 1978.
sundaralingam2023curobo — Sundaralingam et al., cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation, 2023.
gridfinity — Z. Freedman, Gridfinity (web).
-->
$$

# Appendix A: Parameters {#sec:appendix-params}

TODO: complete from facts.md §4, using the values in the lab's `station.local.json`.

| Item | Marker |
|---|---|
| Grid tiles | ArUco `DICT_4X4_50`, IDs 0, 1, 2, side $s_m = 32$ mm |
| Cube | ArUco `DICT_ARUCO_ORIGINAL`, ID 5, side $s_c = 37$ mm (TODO(lab): measure) |

: Fiducial markers. {#tbl:markers}
