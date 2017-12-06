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

% Variable initialization
no_of_states = size(G,1);
no_of_controls = size(G,2);
expected_value_eval = 0;
expected_value_improv = 0;
count = 0;
% True as soon as the computed J_opt equals the J_opt we started from
abort_flag = false;

% Temporary array to hold updated cost for current policy (J_mu^h(i))
policy_eval_cost = ones(1,no_of_states);
J_opt = policy_eval_cost;
temporary_cost = zeros(1,no_of_controls);

% Start with random policy, e.g. 1 (stay) as it's allowed for all states
u_opt_ind = ones(1,no_of_states);

% Iterate
while ~abort_flag
    % First iteration still needs to compute J_opt
    if count~=0
        J_previous = J_opt;
    end
    
    % Policy Evaluation
    for i=1:no_of_states
        for j=1:no_of_states
            % P(i,j,u) is 0 for invalid u, right?
            expected_value_eval = expected_value_eval + ...
                P(i,j,u_opt_ind(i))*J_opt(j);
        end
        
        policy_eval_cost(i) = G(i,u_opt_ind(i))+expected_value_eval;
        expected_value_eval = 0;
        
    end
    
    J_opt = policy_eval_cost;
    
    % Abort when the computed cost equals the previous one, first iteration
    % always false because no J_previous available
    if count==0
        abort_flag = false;
    else
        abort_flag = isequal(J_opt,J_previous);
        % Print out cost difference for info
%         diff = "Difference between previous and current cost is %d\n";
%         diff_str = sprintf(diff, norm((J_previous-J_opt)));
%         fprintf(diff_str)
    end
    
    % Store current cost to be compared for aborting while loop
    J_previous = J_opt;
    count = count+1;
    
    % Policy Improvement
    for i=1:no_of_states
        for u=1:no_of_controls % should be allowed control inputs
            for j=1:no_of_states
                % P(i,j,u) is 0 for invalid u, right?
                expected_value_improv = expected_value_improv + ...
                    P(i,j,u)*J_opt(j);
            end
            % G(i,u) is inf for invalid u, right?
            temporary_cost(u) = G(i,u)+expected_value_improv;
            expected_value_improv = 0;
        end
        
        % Determine u (index) corresponding to minimum cost
        [min_cost, min_cost_index] = min(temporary_cost);
        u_opt_ind(i) = min_cost_index(1);
    end
end
    
disp('Number of policy iterations:' + count);

end

