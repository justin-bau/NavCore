module NavCore

using LinearAlgebra
using Random

include("system_operations.jl")
include("data_generation.jl")
include("kalman_filter.jl")

export discretize_state_equations, discretize_process_noise,
       discretize_measurement_equations,
       generate_constant_signal, generate_white_noise, produce_measurements, generate_centered_correlated_noise,
       run_kalman_filter_scalar, run_kalman_filter, Kalman_Results, forward_simulate_system, compute_nees, compute_nis

end
