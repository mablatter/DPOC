function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by linear programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (1 x MN) matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (1 x MN) matrix containing the indices of the optimal control
%       	inputs for each element of the state space.

% Variable initialization
no_of_states = size(G,1);
no_of_controls = size(G,2);

u_opt_ind = ones(1,no_of_states);

weights = -ones(1,no_of_states);
A = zeros(no_of_states*no_of_controls,no_of_states);
constraints = zeros(no_of_states*no_of_controls,1);

% Rewrite problem such that it becomes a linear program (matrix only)
for u=1:no_of_controls
    start_index = no_of_states*(u-1)+1;
    end_index = no_of_states*u;
    A(start_index:end_index,:) = eye(no_of_states) - P(:,:,u);
    constraints(start_index:end_index,:) = G(:,u);
end
% Replaces inf with a large number (linprog cannot handle inf)
constraints(constraints==inf) = 1000000;

J_opt = linprog(weights',A,constraints);

%Find optimal policy
for i=1:no_of_states
    for u=1:no_of_controls
        expected_value = G(i,u);
        for j=1:no_of_states
            expected_value = expected_value + P(i,j,u)*J_opt(j);
        end
        %HACK
        if abs(expected_value-J_opt(i)) < 0.00001
            u_opt_ind(i) = u;
        end
    end
end

