function G = ComputeStageCosts( stateSpace, controlSpace, mazeSize, walls, targetCell, holes, resetCell, p_f, c_p, c_r )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   attainable control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, disturbanceSpace,
%   mazeSize, walls, targetCell) computes the stage costs for all states in
%   the state space for all attainable control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (MN x 2) matrix, where the i-th row represents the i-th
%           element of the state space. Note that the state space also
%           contains the target cell, in order to simplify state indexing.
%
%       controlSpace:
%           A (L x 2) matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       mazeSize:
%           A (1 x 2) matrix containing the width and the height of the
%           maze in number of cells.
%
%   	walls:
%          	A (2K x 2) matrix containing the K wall segments, where the start
%        	and end point of the k-th segment are stored in row 2k-1
%         	and 2k, respectively.
%
%    	targetCell:
%          	A (2 x 1) matrix describing the position of the target cell in
%         	the maze.
%       holes:
%         	A (H x 2) matrix containg the H holes of the maze. Each row
%         	represents the position of a hole.
%
%   	resetCell:
%         	A (1 x 2) matrix describing the position of the reset cell in
%           the maze.
%
%       p_f:
%           The probability of falling into a hole when the ball is
%           traversing through or to a cell with a hole
%       
%       c_p:
%           Every time the ball bounces into a wall or boundary, we get this number 
%            of time steps as penalty.
%       c_r:
%           Every time the ball falls into a hole, the ball is set to the reset cell
%           at the beginning of the next stage and we get this number of time steps
%           as additional penalty.
%
%   Output arguments:
%
%       G:
%           A (MN x L) matrix containing the stage costs of all states in
%           the state space for all attainable control inputs. The entry
%           G(i, l) represents the cost if we are in state i and apply
%           control input l.

% put your code here

L= size(controlSpace,1);
MN=size(stateSpace,1);
G=ones(MN,L);

disturbance=[0 0; 1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
Prob_disturb=[1/9 1/9 1/9 1/9 1/9 1/9 1/9 1/9 1/9];

for i = 1 : MN
	for l = 1 : L
        not_valid = 0;
        new_loc=stateSpace(i,:)+controlSpace(l,:);
        if (new_loc(1) > mazeSize(1) || new_loc(1) < 1 || new_loc(2) > mazeSize(2) || new_loc(2) < 1)
            G(i,l) = inf;
            continue;        
        end
        distance=max(max(abs(controlSpace(l,:))));
        end_probability = 1;
        Prev_Pos = stateSpace(i,:);
        if distance > 0    
            step_ball=controlSpace(l,:)./distance;
            for x = 1:distance
                if sum(abs(step_ball))==1
                    if (sum(step_ball)==-1)
                        wall_coincidence=ismember(walls,[Prev_Pos+step_ball; Prev_Pos+step_ball+flip(step_ball)],'rows');
                    else
                        wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(step_ball)],'rows');
                    end
                    wall_belonging=find(wall_coincidence,size(wall_coincidence,1));
                    wall_contiguous = 0;
                    if size(wall_belonging,1) > 1
                        for a = 2:size(wall_belonging,1)
                            if(mod(wall_belonging(a), 2) == 0) && (wall_belonging(a)-wall_belonging(a-1) == 1)
                                wall_contiguous = wall_contiguous+1;
                            end
                        end
                    end
                    if (wall_contiguous > 0) 
                        G(i, l) = inf;
                        not_valid = 1;
                        continue;
                    end
                elseif sum(abs(step_ball))>1
                    if (step_ball(1) == 1)
                        if (step_ball(2) == 1)
                            wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos+[0 -1]; Prev_Pos + [-1 0]; Prev_Pos + [0 1]; Prev_Pos + [1 0]],'rows'); 
                        elseif (step_ball(2) == -1)
                            wall_coincidence=ismember(walls,[Prev_Pos + [0 -1]; Prev_Pos+[0 -2]; Prev_Pos + step_ball; Prev_Pos; Prev_Pos + [-1 -1]],'rows'); 
                        end
                    elseif (step_ball(1) == -1)
                        if (step_ball(2) == 1)
                            wall_coincidence=ismember(walls, [Prev_Pos + [-1 0]; Prev_Pos+[-1 -1]; Prev_Pos + step_ball; Prev_Pos; Prev_Pos + [-2 0]],'rows'); 
                        elseif (step_ball(2) == -1)
                            wall_coincidence=ismember(walls,[Prev_Pos + step_ball; Prev_Pos+[-1 -2]; Prev_Pos + [-2 -1]; Prev_Pos + [-1 0]; Prev_Pos + [0 -1]],'rows'); 
                        end
                    end
                    wall_belonging=find(wall_coincidence,size(wall_coincidence,1));
                    wall_contiguous = 0;
                    if size(wall_belonging,1) > 1
                        for a = 2:size(wall_belonging,1)
                            if(mod(wall_belonging(a), 2) == 0) && (wall_belonging(a)-wall_belonging(a-1) == 1)
                                wall_contiguous = wall_contiguous+1;
                            end
                        end
                    end
                    if (wall_contiguous > 0) 
                        G(i, l) = inf;
                        not_valid = 1;
                        continue;
                    end
                end
                Prev_Pos = Prev_Pos + step_ball;
                hole_at_pos = ismember(Prev_Pos, holes, 'rows');
                if sum(hole_at_pos) > 0
                    G(i,l) = G(i,l) + end_probability*p_f*c_r;
                    end_probability = end_probability*(1 - p_f);
                end
            end
            if not_valid
                G(i,l) = inf;
                continue;
            end
        end
        for a = 1: size(disturbance(:,1))
            next_pos = stateSpace(i,:)+controlSpace(l,:)+ disturbance(a,:);
            if (next_pos(1) > mazeSize(1) || next_pos(1) < 1 || next_pos(2) > mazeSize(2) || next_pos(2) < 1)
                G(i,l) = G(i,l) + end_probability*Prob_disturb(a)*c_p;
                continue;        
            end
            Prev_Pos = stateSpace(i,:)+controlSpace(l,:);
            if sum(abs(disturbance(a, :)))==1

                if (sum(disturbance(a, :))==-1)
                    wall_coincidence=ismember(walls,[Prev_Pos+disturbance(a, :); Prev_Pos+disturbance(a, :)+flip(disturbance(a, :))],'rows');
                else
                    wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(disturbance(a, :))],'rows');
                end
                wall_belonging=find(wall_coincidence,size(wall_coincidence,1));
                wall_contiguous = 0;
                if size(wall_belonging,1) > 1

                    for x = 2:size(wall_belonging,1)
                        if(mod(wall_belonging(x), 2) == 0) && (wall_belonging(x)-wall_belonging(x-1) == 1)
                            wall_contiguous = wall_contiguous+1;
                        end
                    end
                end
                                                       
                if (wall_contiguous > 0) 
                    G(i,l) = G(i,l) + end_probability*Prob_disturb(a)*c_p;
                    continue
                end
            elseif sum(abs(disturbance(a, :)))>1
                if (disturbance(a, 1) == 1)
                    if (disturbance(a, 2) == 1)
                        wall_coincidence=ismember([Prev_Pos; Prev_Pos+[0 -1]; Prev_Pos + [-1 0]; Prev_Pos + [0 1]; Prev_Pos + [1 0]],walls,'rows'); 
                    elseif (disturbance(a, 2) == -1)
                        wall_coincidence=ismember([Prev_Pos + [0 -1]; Prev_Pos+[0 -2]; Prev_Pos + disturbance(a, :); Prev_Pos; Prev_Pos + [-1 -1]],walls,'rows'); 
                    end    
                elseif (disturbance(a, 1) == -1)
                    if (disturbance(a, 2) == 1)
                        wall_coincidence=ismember([Prev_Pos + [-1 0]; Prev_Pos+[-1 -1]; Prev_Pos + disturbance(a, :); Prev_Pos; Prev_Pos + [-2 0]],walls,'rows'); 
                    elseif (disturbance(a, 2) == -1)
                        wall_coincidence=ismember([Prev_Pos + disturbance(a, :); Prev_Pos+[-1 -2]; Prev_Pos + [-2 -1]; Prev_Pos + [-1 0]; Prev_Pos + [0 -1]],walls,'rows');
                    end
                end
               
                if wall_coincidence(1)==1
                    G(i,l) = G(i,l) + end_probability*Prob_disturb(a)*c_p;
                    continue;
                end
            end
            hole_at_pos = ismember(Prev_Pos + disturbance(a, :), holes, 'rows');
            if sum(hole_at_pos) > 0
                G(i,l) = G(i,l) + end_probability*p_f*c_r*Prob_disturb(a);
            end
      end
   end
end

