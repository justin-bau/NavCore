using NavCore
using Plots
using Random

Random.seed!(1)

Δt = 0.01
N = 50; x_true = 1.0
x0 = 4.4; P0 = 1.
F = 1.; H = 1.;
Q = 0.01; R = 0.2

z   = produce_measurements(generate_constant_signal(x_true, N), generate_white_noise(0, R, N))
res = run_kalman_filter_scalar(x0, P0, F, H, Q, R, z)

p1 = plot(1:N, [res.P_estimated[2:N+1], R./(1:N), 1 ./ (1/P0 .+ (1:N)/R)], 
     label=["P_estimated" "R/n" "1/(1/P0 + n/R)"])

p2 = plot(0:N, [x_true*ones(N+1), [x_true; z], res.x_estimated],
     label=["x_true" "measurements" "filtered_data"])

plot(p1, p2, layout=(2, 1))
