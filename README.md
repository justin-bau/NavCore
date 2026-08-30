[![CI](https://github.com/justin-bau/NavCore/actions/workflows/CI.yml/badge.svg)](https://github.com/justin-bau/NavCore/actions/workflows/CI.yml)
# NavCore

GNSS/INS sensor fusion from scratch in Julia as a learning project in both Julia and
estimation/navigation. It also makes use of Julia's tolerance of unicode characters, this could
potentially mean that the code does not display without some fonts.

This is a work in progress I do when I can find some time.

The goal being to actually understand as I'm coding, the filter is built gradually:

1. ~~Scalar Kalman filter (constant estimation)~~
2. ~~Linear KF, vector state (constant-velocity tracking)~~ + consistency checks (NEES/NIS) (halfway done). 
3. EKF + attitude (quaternions, error state).
4. Simulate measurements from an IMU with a real datasheet error model.
5. Build a comparison between EKF, ESKF and UKF.
6. Add GNSS measurements for sensor fusion.

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
