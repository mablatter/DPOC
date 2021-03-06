% main.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2017
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Rajan Gill, Weixuan Zhang
%

%% clear workspace and command window
clear all;
close all;
clc;

%% define parameters for dynamics
p_f = 0.6; % probability of falling down a hole 

max_steps = 2; % maximum number of cells we can traverse in one time step.

%% define wall and hole penalty
c_p = 5;
% Every time the ball bounces into a wall or boundary, we get this number 
% of time steps as penalty.
c_r = 2;
% Every time the ball falls into a hole, the ball is set to the reset cell
% at the beginning of the next stage and we get this number of time steps
% as additional penalty.

%% define problem size and generate maze
shouldGenerateMaze = true;
if shouldGenerateMaze
	mazeSize = [ 12, 14 ]; % N, M
	[ walls, targetCell, holes, resetCell ] = GenerateMaze( mazeSize( 1 ), ...
        mazeSize( 2 ), true );
    % This generates a new random maze.
else
    load( 'pregeneratedMaze.mat' );
    % In order to save time we can just load a pre-generated maze.
end
PlotMaze( 1, mazeSize, walls, targetCell, holes, resetCell );

%% generate control space
controlSpace = [0 0];
u_hat = [1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
for u_step = 1 : max_steps
    controlSpace = [controlSpace; u_hat*u_step];
end
% This generates the general (L x 2) matrix 'controlSpace', where the l-th row represents
% the l-th element of the control space. Note that the control space U(i)
% for a particular state i might be reduced due to the walls of the maze.

%% generate state space
stateSpace = [];
for i = 1 : mazeSize( 1 )
    for j = 1 : mazeSize( 2 )
        index = ( i - 1 ) * mazeSize( 2 ) + j;
        stateSpace( index, : ) = [ i, j ];
    end
end
% This generates a (MN x 2) matrix 'stateSpace', where each row represents
% an element of the state space. Note that the stateSpace also included the
% termination state for simplicity with indexing.

%% compute transition probabilities
P = ComputeTransitionProbabilities( stateSpace, controlSpace, ...
    mazeSize, walls, targetCell, holes, resetCell, p_f );
% This computes the transition probabilities between all states in the
% state space for all attainable control inputs.
% The transition probability matrix has the dimension (MN x MN x L), i.e.
% the entry P(i, j, l) representes the transition probability from state i
% to state j if control input l is applied.
% If a control input l is not feasible for a particular state i, the
% transition  probabilities to all states can be set to zero.

%% compute stage costs
G = ComputeStageCosts( stateSpace, controlSpace, ...
    mazeSize, walls, targetCell, holes, resetCell, p_f, c_p, c_r );
% This computes the stage costs for all states in the state space for all
% attainable control inputs.
% The stage cost matrix has the dimension (MN x L), i.e. the entry G(i, l)
% represents the cost if we are in state i and apply control input l.
% If a control input l is not feasible for a particular state i, the stage
% cost can be set to infinity.

%% solve stochastic shortest path problem
% Here we solve the stochastic shortest path problem by Value Iteration,
% Policy Iteration, and Linear Programming.
% VI
[ J_opt_vi, u_opt_ind_vi ] = ValueIteration( P, G );

figH = PlotMaze( 2, mazeSize, walls, targetCell, holes, resetCell, stateSpace, ...
    controlSpace, J_opt_vi, u_opt_ind_vi );
figure(figH);
title(strcat('Value iteration (width=', num2str(mazeSize(1)), ', height=', num2str(mazeSize(2)), ')'));

%% PI
[ J_opt_pi, u_opt_ind_pi ] = PolicyIteration( P, G );

figH = PlotMaze( 3, mazeSize, walls, targetCell, holes, resetCell, stateSpace, ...
    controlSpace, J_opt_pi, u_opt_ind_pi );
figure(figH);
title(strcat('Policy iteration (width=', num2str(mazeSize(1)), ', height=', num2str(mazeSize(2)), ')'));

%% LP
[ J_opt_lp, u_opt_ind_lp ] = LinearProgramming( P, G );

figH = PlotMaze( 4, mazeSize, walls, targetCell, holes, resetCell, stateSpace, ...
    controlSpace, J_opt_lp, u_opt_ind_lp );
figure(figH);
title(strcat('Linear programming (width=', num2str(mazeSize(1)), ', height=', num2str(mazeSize(2)), ')'));

%% check if all J_opt and u_opt are same
same = true;
accuracy = 0.00001;
if ~(norm(J_opt_vi-J_opt_pi)<accuracy)
    disp('J_opt_vi not equal J_opt_pi')
    same = false;
end
if ~(norm(J_opt_vi-J_opt_lp')<accuracy)
    disp('J_opt_vi not equal J_opt_lp')
    same = false;
end
if ~(norm(J_opt_pi-J_opt_lp')<accuracy)
    disp('J_opt_pi not equal J_opt_lp')
    same = false;
end 
if same
    disp('Optimal costs are identical')
end
same = true;

if ~isequal(u_opt_ind_vi,u_opt_ind_pi)
    disp('Optimal policies vi and pi are not identical')
    same = false;
end
if ~isequal(u_opt_ind_lp,u_opt_ind_pi)
    disp('Optimal policies lp and pi are not identical')
    same = false;
end
if ~isequal(u_opt_ind_lp,u_opt_ind_vi)
    disp('Optimal policies lp and vi are not identical')
    same = false;
end
if same
    disp('Optimal policies are identical')
end

%% display that terminated
disp('terminated');
