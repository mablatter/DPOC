function P = ComputeTransitionProbabilities( stateSpace, controlSpace, mazeSize, walls, targetCell, holes, resetCell, p_f )

%%Control if we reach to the target cell to not move.


%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all attainable control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   disturbanceSpace, mazeSize, walls,Z targetCell) computes the transition
%   probabilities between all states in the state space for all attainable
%   control inputs.
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
%       
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
%   Output arguments:
%
%       P:
%           A (MN x MN x L) matrix containing the transition probabilities
%           between all states in the state space for all attainable
%           control inputs. The entry P(i, j, l) represents the transition
%           probability from state i to state j if control input l is
%           applied.

% outer_boundaries=[];
% 
% for i=0:mazeSize(1)
%     for j=0:mazeSize(2)-1
%         if i==0 ||i==mazeSize(1)
%             outer_boundaries=[outer_boundaries; [i j];[i j+1]];
%         elseif i>0
%             outer_boundaries=[outer_boundaries; [i-1 0];[i 0] ;[i-1 6];[i 6]];
%         end
%     end
% end
% 
% walls=[walls; outer_boundaries]


L= size(controlSpace,1);
MN=size(stateSpace,1);
disturbance=[0 0; 1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];
Prob_disturb=[1/9 1/9 1/9 1/9 1/9 1/9 1/9 1/9 1/9];

P=zeros(MN,MN,L);
r= sub2ind(flip(mazeSize),resetCell(2),resetCell(1));

termination_index= sub2ind(flip(mazeSize),targetCell(2),targetCell(1));


for i = 1 :  MN
    adapted_controlSpace=[];
	if i==termination_index
    else
        for control=1:L
            Prev_Pos=stateSpace(i,:);
            distance=max(max(abs(controlSpace(control,:))));
            if ~distance
                adapted_controlSpace= [adapted_controlSpace;  controlSpace(control,:)];
            else
                step_ball=controlSpace(control,:)./distance;
                for p=1:distance
                    Next_Pos=Prev_Pos+step_ball; 
                    if Next_Pos(1)>mazeSize(1) || Next_Pos(2)>mazeSize(2) || Next_Pos(1)<1  || Next_Pos(2)<1
                        break
                    elseif sum(abs(step_ball))==1
                        if (sum(step_ball)==-1)
                            wall_coincidence=ismember(walls,[Prev_Pos+step_ball; Prev_Pos + [-1 -1]],'rows');
                        else
                            wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(step_ball)],'rows');
                        end
                        wall_belonging=find(wall_coincidence,size(wall_coincidence,1));
                        wall_contiguous =0;
                        if size(wall_belonging,1) > 1
                            for a = 2:size(wall_belonging,1)
                                if(mod(wall_belonging(a), 2) == 0) && (wall_belonging(a)-wall_belonging(a-1) == 1)
                                    wall_contiguous = wall_contiguous+1;
                                end
                            end
                        end
                        if wall_contiguous
                            break
                        end

                    elseif sum(abs(step_ball))>1
                        vector_check=step_ball;
                        vector_check(vector_check>0)=0;
                        wall_coincidence=ismember(walls,[Prev_Pos+vector_check; Prev_Pos+vector_check],'rows');

                        if sum(wall_coincidence)
                            break
                        end
                    end
                    Prev_Pos=Next_Pos;
                    if p==distance  %If we reach the virtual destination without crossing walls, it is a valid control input.
                        adapted_controlSpace= [adapted_controlSpace;  controlSpace(control,:)];
                    end
                end
            end
        end
    

        for control = 1:size(adapted_controlSpace,1)
            Control_input=find(controlSpace(:, 1) == adapted_controlSpace(control,1) & controlSpace(:, 2) == adapted_controlSpace(control,2));
            Prev_Pos=stateSpace(i,:);
            X_Prev=i;
            distance=max(max(abs(adapted_controlSpace(control,:))));
            if ~distance
                P(X_Prev,X_Prev,control)= 1;
            else
                step_ball=adapted_controlSpace(control,:)./distance;
                starting_probability=1;
                cumulative_probability=starting_probability;
                for p=1:distance
                    Next_Pos=Prev_Pos+step_ball;
                    X= sub2ind(flip(mazeSize),Next_Pos(2),Next_Pos(1));
                        %Hole detection
%                     if p==1
%                         if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
%                             %ismember(controlSpace,adapted_controlSpace(l,:),'rows'))
%                             P(i,r,Control_input)= p_f;
%                             P(i,X,Control_input)= (1-p_f); 
%                         else
%                             P(i,X,Control_input)= 1;
%                         end
%                     end
% 
%                     if p>1
%                         X_Prev= sub2ind(flip(mazeSize),Prev_Pos(2),Prev_Pos(1));
%                         if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
%                             P(i,r,Control_input)= P(i,r,Control_input)+p_f*(P(i,X_Prev,Control_input));%%No. P(i_State,X,l)=0
%                             P(i,X,Control_input)= (1-p_f)*P(i,X_Prev,Control_input); 
%                             P(i,X_Prev,Control_input)=0;
%                         end
%                         if ~(X==X_Prev)
%                             P(i,X,Control_input)= P(i,X_Prev,Control_input)+P(i,X,Control_input); 
%                             P(i,X_Prev,Control_input)= 0;
%                         end  
%                     end   
%                     i_State=X;
%                     Prev_Pos=Next_Pos; 
                
                    if p>1
                        P(i,X_Prev,Control_input)=P(i,X_Prev,Control_input)-cumulative_probability;
                    end
                    
                    if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
                        P(i,r,Control_input)= p_f*cumulative_probability+P(i,r,Control_input);
                        cumulative_probability=(1-p_f)*cumulative_probability;
                    end
                    
                    P(i,X,Control_input)= cumulative_probability+P(i,X,Control_input);
                    
                    X_Prev=X;
                    Prev_Pos=Next_Pos; 
                end
            end                        

           %Disturbance
            prior_prob=P(i,X_Prev,Control_input);
            for dist = 1:size(disturbance,1)
                Next_Pos=Prev_Pos+disturbance(dist,:);                  
                if Next_Pos(1)>mazeSize(1) || Next_Pos(2)>mazeSize(2) || Next_Pos(1)<1  || Next_Pos(2)<1
                    Next_Pos=Prev_Pos;
                elseif sum(abs(disturbance(dist,:)))==1
                    if (sum(disturbance(dist,:))==-1)
                        wall_coincidence=ismember(walls,[Prev_Pos+disturbance(dist,:); Prev_Pos+disturbance(dist,:)+flip(disturbance(dist,:))],'rows');
                    else
                        wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(disturbance(dist,:))],'rows');
                    end
                    wall_belonging=find(wall_coincidence,size(wall_coincidence,1));
                    wall_contiguous =0;
                    if size(wall_belonging,1) > 1
                        for a = 2:size(wall_belonging,1)
                            if(mod(wall_belonging(a), 2) == 0) && (wall_belonging(a)-wall_belonging(a-1) == 1)
                                wall_contiguous = wall_contiguous+1;
                            end
                        end
                    end
                    if wall_contiguous
                    %if (size(wall_contiguous,1)>1) && sum(wall_contiguous(1:size(wall_contiguous,2)))>1
                        Next_Pos=Prev_Pos;
                    end
                elseif sum(abs(disturbance(dist,:)))>1
                    vector_check=disturbance(dist,:);
                    vector_check(vector_check>0)=0;
                    wall_coincidence=ismember(walls,[Prev_Pos+vector_check; Prev_Pos+vector_check],'rows');
                %                             wall_belonging=find(wall_coincidence,size(wall_coincidence,1))
                %                             wall_contiguous=diff(wall_belonging)==1
                    if sum(wall_coincidence)
                        Next_Pos=Prev_Pos;
                    end
                end

                X= sub2ind(flip(mazeSize),Next_Pos(2),Next_Pos(1));

                %Hole detection
                joint_prob=prior_prob*Prob_disturb(dist);

                X_Prev= sub2ind(flip(mazeSize),Prev_Pos(2),Prev_Pos(1));

                if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
                    P(i,r,Control_input)= P(i,r,Control_input)+p_f*joint_prob;
                    P(i,X,Control_input)= (1-p_f)*joint_prob;
                elseif X==X_Prev && sum(abs(disturbance(dist,:)))
                    P(i,X,Control_input)=P(i,X,Control_input)+joint_prob;
                else
                    P(i,X,Control_input)= joint_prob;
                end
            end
        end
	end
end

