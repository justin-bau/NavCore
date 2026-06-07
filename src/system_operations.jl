function discretize_state_equations(A, B, Δt)
  n = size(A, 1)
  p = size(B, 2)
  to_discretize = [A B; zeros(p, n + p)] * Δt

  result = exp(to_discretize)
  F = result[1:n, 1:n]
  G = result[1:n, n+1:n+p]
  F, G
end

function discretize_measurement_equations(C, D, Δt)
  C, D
end

function discretize_process_noise(A, Q, Δt)
  n = size(A, 1)
  F = [-A Q; zeros(n,n) A'] * Δt
  G = exp(F)

  G[n+1:2n, n+1:2n]' * G[1:n, n+1:2n]
end

