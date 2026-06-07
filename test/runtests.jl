using Test
using NavCore
using LinearAlgebra
using Random

Random.seed!(1)

@testset "NavCore" begin
  @testset "scalar KF; constant; Q=0" begin
    R = 0.2; N = 50; x_true = 1.0
    x0 = 4.4; P0 = 1.
    F = 1; H = 1; Q = 0.
    z   = produce_measurements(generate_constant_signal(x_true, N), generate_white_noise(0, R, N))
    res = run_kalman_filter_scalar(x0, P0, F, H, Q, R, z)
  
    for i in 1:N
        @test res.P_estimated[i+1] ≈ 1/(1/P0 + i/R) rtol=1e-6
    end
  end
  
  @testset "Verifying discretization" begin
    # Test the functions against a known simple integrator system
    Δt = 0.1
    A = [0. 1.; 0. 0.]; B = [0.; 1.]
    F = [1. Δt; 0. 1.]; G = [Δt^2/2.; Δt]
    Fd, Gd = discretize_state_equations(A, B, Δt)
    @test F ≈ Fd rtol=1e-6
    @test G ≈ Gd rtol=1e-6
  
    q = 0.1; Q = [0. 0.; 0. q]
    Qd_expected = q * [Δt^3/3. Δt^2/2.;
                       Δt^2/2. Δt]
    @test Qd_expected ≈ discretize_process_noise(A, Q, Δt) rtol=1e-6
  end
end
