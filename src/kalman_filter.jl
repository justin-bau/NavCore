struct Kalman_Results
  x_predicted::Array{Float64,2}
  P_predicted::Array{Float64,3}
  x_estimated::Array{Float64,2}
  P_estimated::Array{Float64,3}
  K::Array{Float64,3}
  v::Array{Float64,2}
  S::Array{Float64,3}
  ϵ_v::Vector{Float64}
  Kalman_Results(n::Int64, m::Int64, N::Int64) = new(zeros(n, N), zeros(n, n, N), zeros(n, N + 1), zeros(n, n, N + 1), zeros(n, m, N), zeros(m, N), zeros(m, m, N), zeros(N))
end

function Kalman_Results(x0, P0, m::Int64, N::Int64)
  r = Kalman_Results(length(x0), m, N)
  r.x_estimated[:, 1] .= x0
  r.P_estimated[:, :, 1] .= P0
  r
end

function run_kalman_filter_scalar(x0, P0, F, H, Q, R, z)
  N = length(z)
  results = Kalman_Results(x0, P0, 1, N)
  for i = 1:N
    # Prediction step
    results.x_predicted[i] = F * results.x_estimated[i]
    results.P_predicted[i] = F * results.P_estimated[i] * F' + Q

    # Update step (Joseph form)
    results.K[i] = results.P_predicted[i] * H' / (H * results.P_predicted[i] * H' + R)
    results.x_estimated[i+1] = results.x_predicted[i] + results.K[i] * (z[i] - H * results.x_predicted[i])
    results.P_estimated[i+1] = (1 - results.K[i] * H) * results.P_predicted[i] * (1 - results.K[i] * H)' + results.K[i] * R * results.K[i]'
  end
  results
end

function run_kalman_filter(x0, P0, A, B, C, D, Q, R, z, Δt)
  N = size(z, 2) - 1
  results = Kalman_Results(x0, P0, size(z, 1), N)

  #TODO: decide on whether to discretize inside the filter, or outside
  # -> probably more principled to do it outside
  F, G = discretize_state_equations(A, B, Δt)
  H, Dd = discretize_measurement_equations(C, D, Δt)
  # R is assumed already discretized, coming from numerical sensors
  Q = discretize_process_noise(A, Q, Δt)

  for i = 1:N
    # Prediction step
    results.x_predicted[:, i] = F * results.x_estimated[:, i]
    results.P_predicted[:, :, i] = F * results.P_estimated[:, :, i] * F' + Q

    # Update step (Joseph form)
    results.v[:, i] = z[:, i+1] - H * results.x_predicted[:, i]
    results.S[:, :, i] = H * results.P_predicted[:, :, i] * H' + R
    results.K[:, :, i] = results.P_predicted[:, :, i] * H' / results.S[:, :, i]
    results.x_estimated[:, i+1] = results.x_predicted[:, i] + results.K[:, :, i] * results.v[:, i]
    results.P_estimated[:, :, i+1] = ((I - results.K[:, :, i] * H) * results.P_predicted[:, :, i] * (I - results.K[:, :, i] * H)'
                                      +
                                      results.K[:, :, i] * R * results.K[:, :, i]')
    results.ϵ_v[i] = (results.v[:, i]'/results.S[:, :, i]*results.v[:, i])[1]
  end
  results
end

function compute_nees(x_true, x_estimated, P_estimated)
  x_err = x_true - x_estimated
  x_err' / P_estimated * x_err
end

function compute_nis(z, H, x_predicted, S)
  v = z - H * x_predicted
  v' / S * v
end
