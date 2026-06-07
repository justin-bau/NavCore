using NavCore
using Plots
using Random

Random.seed!(1)

Δt = 0.01
N = 300

A = [0. 1.; 0. 0.]
B = [0.; 1.]
C = [1. 0.]
D = [0.]

R = [0.05;;]
Q = [0.01 0.; 0. 0.02]

u = zeros(size(B, 2), N)
x0_true = [0, -0.3]
x0_initial_guess = [0.05; 0.5]
P0 = [0.1 0; 0 1]
x_true, y_true = forward_simulate_system(A, B, C, D, x0_true, N, u, Δt)
z = y_true + generate_centered_correlated_noise(R, N + 1)

results = run_kalman_filter(x0_initial_guess, P0, A, B, C, D, Q, R, z, Δt)

ϵ = zeros(N)
for i = 1:N
  ϵ[i] = compute_nees(x_true[:,i], results.x_estimated[:,i], results.P_estimated[:, :, i])  
end

p1 = plot(0:N, [results.P_estimated[1, 1, :], results.P_estimated[1, 2, :], results.P_estimated[2, 2, :]], 
          label=["P(1,1)" "P(1,2)" "P(2,2)"])

p2 = plot(0:N, [x_true[1, :], results.x_estimated[1, :], z[:]],
     label=["position_true" "position_filtered" "measurements"])

p3 = plot(0:N, [x_true[2, :], results.x_estimated[2, :]],
     label=["velocity_true" "velocity_filtered"])

p4 = plot(1:N, [fill(length(x0_true), N), results.ϵ_v, ϵ],
     label=["degrees of freedom" "NIS" "NEES"])

plot(p1, p2, p3, p4, layout=(2, 2))
