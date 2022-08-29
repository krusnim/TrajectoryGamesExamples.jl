using TrajectoryGamesBase:
    TrajectoryGamesBase, TrajectoryGame, RecedingHorizonStrategy, num_players, rollout
using TrajectoryGamesExamples: TestUtils, animate_sim_steps, three_player_meta_tag
using LiftedTrajectoryGames: LiftedTrajectoryGameSolver

using Test: @testset
using BlockArrays: mortar
using GLMakie: GLMakie
using Makie: Makie
using Random: MersenneTwister


game = three_player_meta_tag()
initial_state = mortar([[-1.0, 0.0, 0.0, 0.0], [1.0, -1.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0]])
planning_horizon = 20
rng = MersenneTwister(1)
turn_length = 10

solver = LiftedTrajectoryGameSolver(game, planning_horizon)
receding_horizon_strategy = RecedingHorizonStrategy(; solver, game, turn_length = 10)
sim_steps = rollout(
    game.dynamics,
    receding_horizon_strategy,
    initial_state,
    500;
    get_info = (γ, x, t) -> γ.receding_horizon_strategy,
)

animate_sim_steps(game, sim_steps; live = false, framerate = 30)