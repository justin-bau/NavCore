# nav-core

GNSS/INS sensor fusion from scratch in Julia as a learning project in both Julia and
estimation/navigation.

Goal being to actually understand, the filter is built gradually:

1. Scalar Kalman filter (constant estimation) — get a feel for P, Q, R.
2. Linear KF, vector state (constant-velocity tracking) + consistency checks (NEES/NIS).
3. EKF + attitude (quaternions, error state).
4. IMU mechanization + a real datasheet error model.
5. Loosely- then tightly-coupled ESKF, and an observability study.

Scaffolding of the project (repository structure, pkgs) conceived with AI. Actual code is
hand-written.

## Running

```text
julia> ]               # package mode
pkg> activate .
pkg> instantiate       # install deps from Project.toml
```

Then from the REPL (Revise recommended):

```text
julia> using Revise
julia> includet("scripts/layer1_constant.jl")
```

Tests: pkg> test

## References

- Faragher (2012), intuitive KF derivation.
- Peter D. Joseph, Richard S. Bucy (1968),
  *Filtering for Stochastic Processes with Applications to Guidance.* -> on Joseph form for Kalman
  filtering
- Charles Van Loan: Computing integrals involving the matrix exponential, IEEE Transactions on
  Automatic Control. 23 (3): 395–404, 1978 -> on discretization of a continuous noisy system
- Bar-Shalom, Li & Kirubarajan, *Estimation with Applications to Tracking and Navigation* (KF +
  consistency tests).
- Groves, *Principles of GNSS, INS, and Multisensor Integrated Navigation*.
- Solà, *Quaternion kinematics for the error-state Kalman filter*.
