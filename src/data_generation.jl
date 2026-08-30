function forward_simulate_system(A, B, C, D, X0, N, u, Δt)
  F, G = discretize_state_equations(A, B, Δt)
  H, Ψ = discretize_measurement_equations(C, D, Δt)

  X = zeros(length(X0), N + 1)
  Y = zeros(size(H, 1), N + 1)

  X[:, 1] = X0
  Y[:, 1] = H * X[:, 1]

  for i = 1:N
    X[:, i+1] = F * X[:, i] + G * u[:, i]
    Y[:, i+1] = H * X[:, i+1]
  end
  X, Y
end

function generate_constant_signal(value, N)
  fill(value, N)
end

function generate_white_noise(μ, R, N)
  μ .+ sqrt(R) .* randn(N)
end

function generate_centered_correlated_noise(R, N)
  C = cholesky(R)
  C.L * randn(size(R, 1), N)
end

function produce_measurements(signal, noise)
  signal .+ noise
end
