function P = ComputeTransitionProbabilities( stateSpace, controlSpace, mazeSize, walls, targetCell, holes, resetCell, p_f )

%%Control if we reach to the target cell to not move.


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

P=zeros(MN,MN,L);
r= sub2ind(flip(mazeSize),resetCell(2),resetCell(1));

for l = 1:L
	for i = 1 :  MN
        for j = 1 : MN
            out_bounds=stateSpace(i,:)+controlSpace(l,:);
            if sum((stateSpace(j,:)-stateSpace(i,:))==controlSpace(l,:))==2|| out_bounds(1)>mazeSize(1) || out_bounds(2)>mazeSize(2) || out_bounds(1)<1  || out_bounds(2)<1
                distance=max(max(abs(controlSpace(l,:))));
                i_State=i;
                if l==10 && i==1
                    a=1;
                end
                if ~distance && i_State==j
                    P(i_State,j,l)= 1;
                else
                    step_ball=controlSpace(l,:)./distance;
                    for p=1:distance
                        Prev_Pos=stateSpace(i_State,:);
                        Next_Pos=Prev_Pos+step_ball; 
                        
                        if Next_Pos(1)>mazeSize(1) || Next_Pos(2)>mazeSize(2) || Next_Pos(1)<1  || Next_Pos(2)<1
                            Next_Pos=Prev_Pos;
                        elseif sum(abs(step_ball))==1
                            if (sum(step_ball)==-1)
                                wall_coincidence=ismember(walls,[Prev_Pos+step_ball; Prev_Pos+step_ball+flip(step_ball)],'rows');
                            else
                                wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(step_ball)],'rows');
                            end
                            wall_belonging=find(wall_coincidence,size(wall_coincidence,1));
                            wall_contiguous=diff(wall_belonging)==1;
                            if sum(wall_contiguous)
                            %if (size(wall_contiguous,1)>1) && sum(wall_contiguous(1:size(wall_contiguous,2)))>1
                                    Next_Pos=Prev_Pos;
                            end
                        elseif sum(abs(step_ball))>1
                            vector_check=step_ball;
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
                        if p==1
                            if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
                                P(i,r,l)= p_f;
                                P(i,X,l)= (1-p_f); 
                            else
                                P(i,X,l)= 1;
                            end
                        end
                        
                        if p>1
                            X_Prev= sub2ind(flip(mazeSize),Prev_Pos(2),Prev_Pos(1));
                            if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
                                P(i,r,l)= P(i,r,l)+p_f*(P(i,X_Prev,l));%%No. P(i_State,X,l)=0
                                P(i,X,l)= (1-p_f)*P(i,X_Prev,l); 
                                P(i,X_Prev,l)=0;
                            end
                            if ~(X==X_Prev)
                                P(i,X,l)= P(i,X_Prev,l)+P(i,X,l); 
                                P(i,X_Prev,l)= 0;
                            end  
                        end      
                    i_State=X;
                    Prev_Pos=Next_Pos; 
                    end
                end                        
            end
        end
	end
%             if (stateSpace(j,:)-stateSpace(i,:))==controlSpace(l,:)

end

%Disturbance

disturbance=[0 0; 1 0; 1 1; 0 1; -1 1; -1 0; -1 -1; 0 -1; 1 -1];

% 
% for i = 1 :  MN
%      for j = 1 : MN
%          for l = 1:size(disturbance,1)
%              Next_Pos=Prev_Pos+disturbance(l,:)                   
%                         if Next_Pos(1)>mazeSize(1) || Next_Pos(2)>mazeSize(2) || Next_Pos(1)<1  || Next_Pos(2)<1
%                             Next_Pos=Prev_Pos;
%                         elseif sum(abs(disturbance(l,:)))==1
%                             if (sum(disturbance(l,:))==-1)
%                                 wall_coincidence=ismember(walls,[Prev_Pos+disturbance(l,:); Prev_Pos+disturbance(l,:)+flip(disturbance(l,:))],'rows');
%                             else
%                                 wall_coincidence=ismember(walls,[Prev_Pos; Prev_Pos-flip(disturbance(l,:))],'rows');
%                             end
%                             wall_belonging=find(wall_coincidence,size(wall_coincidence,1));
%                             wall_contiguous=diff(wall_belonging)==1;
%                             if sum(wall_contiguous)
%                             %if (size(wall_contiguous,1)>1) && sum(wall_contiguous(1:size(wall_contiguous,2)))>1
%                                     Next_Pos=Prev_Pos;
%                             end
%                         elseif sum(abs(disturbance(l,:)))>1
%                             vector_check=disturbance(l,:);
%                             vector_check(vector_check>0)=0;
%                             wall_coincidence=ismember(walls,[Prev_Pos+vector_check; Prev_Pos+vector_check],'rows');
%     %                             wall_belonging=find(wall_coincidence,size(wall_coincidence,1))
%     %                             wall_contiguous=diff(wall_belonging)==1
%                             if sum(wall_coincidence)
%                                     Next_Pos=Prev_Pos;
%                             end
%                         end
% 
%                         X= sub2ind(flip(mazeSize),Next_Pos(2),Next_Pos(1));
% 
%                         %Hole detection
%                         P(Prev_State,X,l)= 1/9;
% 
%                         if ~isempty(holes) && sum(ismember(holes,Next_Pos,'rows'))
%                             P(Prev_State,r,l)= p_f*P(Prev_State,X,l);
%                             P(Prev_State,X,l)= (1-p_f)*P(Prev_State,X,l); 
%                         end
%                         Prev_State=X;
%                         Prev_Pos=Next_Pos;
%          end
%      end
end
