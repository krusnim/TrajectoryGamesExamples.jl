function three_player_meta_tag(;
    n_environment_sides = 5,
    environment_radius = 6,
    coupling_constraints = nothing,
    control_penalty = 0.1,
    dynamics = planar_double_integrator(;
        state_bounds = (; lb = [-Inf, -Inf, -5, -5], ub = [Inf, Inf, 5, 5]),
        control_bounds = (; lb = [-10, -10], ub = [10, 10]),
    ),
    distance_metric = norm,
)
    cost = let
        function stage_cost(x, u, t, context_state)
            x1, x2, x3 = blocks(x)
            u1, u2, u3 = blocks(u)
            c1 = # first evader goal: be far from the pursuer / make the pursuer use lots of control
                sqrt(distance_metric(x1[1:2] - x3[1:2]) + 0.1) + 
                control_penalty * (distance_metric(u1) - distance_metric(u3))
            c2 = # second evader goal: same as first evader with respect to self
                sqrt(distance_metric(x2[1:2] - x3[1:2]) + 0.1) + 
                control_penalty * (distance_metric(u2) - distance_metric(u3))
            c3 = # pursuer goal: be close to one of the evaders - doesn't matter which
                min(c1, c2)
            [-c1, -c2, c3]
        end

        function reducer(scs)
            reduce(.+, scs) ./ length(scs)
        end

        TimeSeparableTrajectoryGameCost(stage_cost, reducer, GeneralSumCostStructure(), 1.0)
    end
    dynamics = ProductDynamics([dynamics for _ in 1:3] |> Tuple)
    env = PolygonEnvironment([[0; 0], [100; 0], [100; 4], [0; 4]])
    TrajectoryGame(dynamics, cost, env, coupling_constraints)
end
