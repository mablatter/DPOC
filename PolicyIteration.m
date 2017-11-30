function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by policy iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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

% put your code here

% Start with random values for policy

% Variable initialization
no_of_states = size(G,1);
no_of_controls = size(G,2);
expected_value = 0;
count = 0;

% Start with random policy, easiest is 0 as its allowed for all states
u_opt_ind = zeros(no_of_states);
temporary_cost = zeros(no_of_controls);

% Iterate
count = count+1;

% Policy Evaluation
for i=1:no_of_states
    for j=1:no_of_states
        % P(i,j,u) is 0 for invalid u, right?
        expected_value = expected_value + P(i,j,u_opt_ind(i))*J_opt(j);
    end
    
    temporary_cost(i) = G(i,u_opt_ind(i))+expected_value;
    expected_value = 0;
end
J_opt = temporary_cost;

% Policy Improvement
% TODO

disp("Number of policy iterations: " + count);

end

