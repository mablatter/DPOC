function P = ComputeTransitionProbabilities( stateSpace, controlSpace, mazeSize, walls, targetCell, holes, resetCell, p_f )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all attainable control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   disturbanceSpace, mazeSize, walls, targetCell) computes the transition
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

L= size(controlSpace,1)
MN=size(stateSpace,1)

P=zeros(MN,MN,L);
r= sub2ind(flip(mazeSize),resetCell(2),resetCell(2));

for l = 1:L
    for i = 1 :  MN
        for j = 1 : MN
            if (stateSpace(j,:)-stateSpace(i,:))==controlSpace(l,:)
                distance=max(max(abs(controlSpace(l,:))));
                if ~distance
                    P(i,j,l)= 1;
                else
                    step_ball=controlSpace(l,:)./distance;
                    Prev_State=i;
                    Prev_Pos=stateSpace(i,:)
                    for p=1:distance
                        Next_Pos=Prev_Pos+step_ball;                       
                        if sum(abs(step_ball))==1
                            if (sum(step_ball)==-1)
                                wall_coincidence=ismember(walls,[Prev_Pos+step_ball; Prev_Pos+step_ball+[0 -1]])
                            else
                                wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(step_ball)],'rows')
                            end
                            Prev_Pos
                            wall_belonging=find(wall_coincidence,size(wall_coincidence,1))
                            wall_contiguous=diff(wall_belonging)==1
                            if sum(wall_contiguous)
                            %if (size(wall_contiguous,1)>1) && sum(wall_contiguous(1:size(wall_contiguous,2)))>1
                                    Next_Pos=Prev_Pos
                                    step_ball  
                            end
%                         elseif sum(abs(step_ball))>1
%                             if 
%                                 wall_coincidence=ismember(walls,[Prev_Pos+step_ball; Prev_Pos+step_ball+[0 -1]])
%                             else
%                                 wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(step_ball)],'rows')
%                             end
%                             Prev_Pos
%                             wall_belonging=find(wall_coincidence,size(wall_coincidence,1))
%                             wall_contiguous=diff(wall_belonging)==1
%                             if sum(wall_contiguous)
%                                     Next_Pos=Prev_Pos
%                                     step_ball  
%                             end
                        end
%                         if sum(abs(step))>=1
%                             if (sum(step)<-1 && ismember(walls,[Prev_Pos+step; Prev_Pos+step+[0 -1]],'rows'))
%                                     Next_Pos=Prev_Pos;
%                             end
%                         end
     
                        X= sub2ind(flip(mazeSize),Next_Pos(2),Next_Pos(1));

                        %Hole detection
                        if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
                            P(Prev_State,X,l)= (1-p_f)^p; 
                            P(Prev_State,r,l)= p_f^p;
                        else
                            P(Prev_State,X,l)= 1;
                        end
                        Prev_State=X;
                        Prev_Pos=Next_Pos;
                    end
                end
            end
        end
    end
end

%Disturbance

disturbance=[0 0; 1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];


% for l = 1:L
%     for i = 1 :  MN
%         for j = 1 : MN
%             if (stateSpace(j,:)-stateSpace(i,:))==controlSpace(l,:)
%                 distance=max(max(abs(controlSpace(l,:))))
%                 for p=1:distance
%                     Pos=stateSpace(i,:)+controlSpace(l,:)*p/distance
%                     r=i+controlSpace(l,1)*M+controlSpace(l,2)
%                     if ismember(holes,stateSpace(r,:)+controlSpace(l,:)*p/distance,'rows')
%                         P(r,j,l)= (1-p_f)^p;
%                         P(
%                     else
%                         P(i,j,l)= 1;
%                     end
%                 end
%             end
%         end
%     end
% end
