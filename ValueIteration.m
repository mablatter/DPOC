function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by value iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
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

% Variable initialization
no_of_states = size(G,1);
no_of_controls = size(G,2);

u_opt_ind = zeros(1,no_of_states);
temporary_cost = zeros(1,no_of_controls);
expected_value = 0;
count = 0;

% Start with random values for costs, J_previous such that abort is false
J_opt = randi(1000,1,no_of_states);
J_previous = J_opt-ones(1,no_of_states);

% Max difference between current and previous cost before aborting
% iteration
abort_threshold = 0.00001;

% Iterate
% max(J_opt-J_previous)>abort_threshold % acc. exercise sheet
while norm(J_opt-J_previous)>abort_threshold
    % Count iterations (just for info) and store current cost to compare
    % later
    count = count+1;
    J_previous = J_opt;
    
    % Computes cost vector for each state by evaluating cost for each
    % control input (i.e. summing stage cost and expected value) and then
    % determining the minimum
    for i=1:no_of_states
        for u=1:no_of_controls
            for j=1:no_of_states
                expected_value = expected_value + P(i,j,u)*J_opt(j);
            end
            temporary_cost(u) = G(i,u)+expected_value;
            expected_value = 0;
        end
        
        % Determine minimum cost and its corresponding u (index)
        [min_cost, min_cost_index] = min(temporary_cost);
        J_opt(i) = min_cost(1);
        u_opt_ind(i) = min_cost_index(1);
        
    end
end

%disp('Number of value iterations:' + count);

end

