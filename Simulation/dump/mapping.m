%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mapping Strategies
%
% For the three end points of the bhand, we have to deliver 3 points 
% from the current human hand configuration. However, for successful 
% matching of the points from the hhand onto possible bhand configurations, 
% the proper choice of the human hand origin is vital.
% 
% The three points of the bhand constitute a triangle. The five fingertips 
% of hhand constitute a pentagon. The needed mapping should not only sort 
% out suiting three points from the pentagon circumference, but also be 
% able to deliver the desired center of gravity of the pentagon. It can be 
% used as a point of reference for the bhand triangle shift in xy plane 
% and/or orientation. The need arises because, the hhand thumb is not placed 
% on the symmetry axis of the digit fingers as is the case in bhand.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% At the first step, the pentagon should be to reduced to a triangle with 
% one of the following strategies:
% 
% 1) using the index, ring and thumb fingertips only, 
%        drawback: the palmar center is to be replaced; for instance with 
%                  the triangle center
% 
% 2) using the index, little and thumb fingertips only, 
%           this can be more stationary as compared to (2), because the 
%           index is stronger than the middle finger
% 
% 3) building a mean from index-middle and ring-little pairs and taking 
% thumb tip as the third point
%           drawback: mean value can follow large jumps because of the 
%                     independent flexion movement of the paired fingers
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% In all of the above, the corresponding tip trajectories should also be
% considered. Because, successive triangles from the hhand deliver also 
% the trajectories for the individual bhand fingertips and the members of 
% these two group should be similar as far as possible.
% 
% For the fingertip trajectorial similarity, the following constraints can be set:
% 
% 1) The bhand nonspread finger flexion occurs in the symmetry plane of the 
% two spread fingers. Therefore, the desired nonspread trajectory from hhand 
% should better lay in the symmetry plane of the two other points. 
% 
% The best matching pentagon-to-triangle strategy for this strategy is (3).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% In any of the strategies, define the symmetry plane of used digits and derive
% the projection of the thumb tip onto that plane to find nonspread desired
% trajectory points.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start the mapping only after the memory elements are full. The system
% saves only 2 past finger tip coordinates. Together with the current 
% points, the loop must have been run at least twice to fill the memory.
% "angle_index" is the iteration index of the control loop, so it gives the
% number of control loop operations.
%
% See "human_control.m". 
if(angle_index >= 3) 
	
	% To set the strategy to use, change "initials_for robotics.m".
	switch triangle_strategy
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        case strategy1          % use index, ring, thumb tips
        
            % Derive three points a,b,c on the digits' symmetry plane:
             a = (hhand_q2cart.index_tip_past2 + hhand_q2cart.ring_tip_past2)/ 2;
             b = (hhand_q2cart.index_tip_past1 + hhand_q2cart.ring_tip_past1)/ 2;
             c = (hhand_q2cart.index_tip + hhand_q2cart.ring_tip)/ 2;
                
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % NOW DERIVE THE SYMMETRIC FRAMES OF REFERENCE at Point c FOR BUILDING OF
                    % A SYMMETRY FRAME OF REFERENCE. 
                    %
                    % The two frames of references have little rotational 
                    c_at_index_tip = hhand.index.tip.A * [c(1) c(2) c(3) 1]';
					A_c1 = trans(c_at_index_tip(1), c_at_index_tip(2), c_at_index_tip(3)) * hhand.index.tip.A;
					
					c_at_ring_tip  = hhand.ring.tip.A * [c(1) c(2) c(3) 1]';
					A_c2 = trans(c_at_ring_tip(1), c_at_ring_tip(2), c_at_ring_tip(3)) * hhand.ring.tip.A;
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					% Build a single frame of reference with a fusion of the two frames of
					% references. The idea is valid, because the fused
					% frames of references are
					A_c = (A_c1 + A_c2)/ 2;
					
					thumb_tip_at_c = A_c * [hhand_q2cart.thumb_tip(1) hhand_q2cart.thumb_tip(2) hhand_q2cart.thumb_tip(3) 1]';
					
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					% Take the projection of the thumb tip on the x-z plane of A_c frame of
					% reference and transform t back to
					projected_thumb_at_c = [thumb_tip_at_c(x_axis)    0    thumb_tip_at_c(z_axis)];
					projected_thumb_4 = inv(A_c) * [ projected_thumb_at_c(1) projected_thumb_at_c(2) projected_thumb_at_c(3) 1]';
					
					projected_thumb = [projected_thumb_4(x_axis) projected_thumb_4(y_axis) projected_thumb_4(z_axis)];
                    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The three points from A = hhand_q2cart.index_tip, B =
            % hhand_q2cart.ring_tip, and C = projected_thumb consitute the
            % triangle to be mapped onto the bhand. We have to derive the
            % heigth, height base, center of gravity of the triangle ABC.
            % THESE ARE THE PARAMETERS TO USE IN BACKWARD SOLUTION:
            %
            corner_A = hhand_q2cart.index_tip;
            corner_B = hhand_q2cart.ring_tip;
            corner_C = projected_thumb;
            
            corner_A_at_c_4     = A_c * [corner_A(1) corner_A(2) corner_A(3)   1]';
            corner_A_at_c       = [corner_A_at_c_4(x_axis)     corner_A_at_c_4(y_axis)   corner_A_at_c_4(z_axis)];
            
            corner_B_at_c_4     = A_c * [corner_B(1) corner_B(2) corner_B(3)   1]';
            corner_B_at_c       = [corner_B_at_c_4(x_axis)     corner_B_at_c_4(y_axis)   corner_B_at_c_4(z_axis)];

            world_origin_at_c_4 = A_c * [0 0 0 1]';
            world_origin_at_c   = [world_origin_at_c_4(x_axis)   world_origin_at_c_4(y_axis)   world_origin_at_c_4(z_axis)];
		
            heigth              = sqrt(  ( c(x_axis) - corner_C(x_axis) )^2 +...
                                         ( c(y_axis) - corner_C(y_axis) )^2 +...
                                         ( c(z_axis) - corner_C(z_axis) )^2); % heigth of the mapping triangle
                                     
            heigth_base_length  = sqrt(  ( corner_A(x_axis) - corner_B(x_axis) )^2 +...
                                         ( corner_A(y_axis) - corner_B(y_axis) )^2 +...
                                         ( corner_A(z_axis) - corner_B(z_axis) )^2 );
                                     
            heigth_base_point   = c;
            
            center_of_gravity   = [ (corner_A(x_axis) + corner_B(x_axis) + corner_C(x_axis))/ 3,...
                                    (corner_A(y_axis) + corner_B(y_axis) + corner_C(y_axis))/ 3,...
                                    (corner_A(z_axis) + corner_B(z_axis) + corner_C(z_axis))/ 3];
    stop = 1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        case strategy2          % use index, little, thumb tips
        
            % Derive three points a,b,c on the digits' symmetry plane:
             a = (hhand_q2cart.index_tip_past2 + hhand_q2cart.little_tip_past2)/ 2;
             b = (hhand_q2cart.index_tip_past1 + hhand_q2cart.little_tip_past1)/ 2;
             c = (hhand_q2cart.index_tip + hhand_q2cart.little_tip)/ 2;
                
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % NOW DERIVE THE SYMMETRIC FRAMES OF REFERENCE at Point c FOR BUILDING OF
                    % A SYMMETRY FRAME OF REFERENCE. 
                    %
                    % The two frames of references have little rotational 
                    c_at_index_tip = hhand.index.tip.A * [c(1) c(2) c(3) 1]';
					A_c1 = trans(c_at_index_tip(1), c_at_index_tip(2), c_at_index_tip(3)) * hhand.index.tip.A;
					
					c_at_little_tip = hhand.little.tip.A * [c(1) c(2) c(3) 1]';
					A_c2 = trans(c_at_little_tip(1), c_at_little_tip(2), c_at_little_tip(3)) * hhand.little.tip.A;
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					% Build a single frame of reference with a fusion of the two frames of
					% references. The idea is valid, because the fused
					% frames of references are
					A_c = (A_c1 + A_c2)/ 2;
					
					thumb_tip_at_c = A_c * [hhand_q2cart.thumb_tip(1) hhand_q2cart.thumb_tip(2) hhand_q2cart.thumb_tip(3) 1]';
                    
					
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					% Take the projection of the thumb tip on the x-z plane of A_c frame of
					% reference and transform t back to
					projected_thumb_at_c = [thumb_tip_at_c(x_axis)    0    thumb_tip_at_c(z_axis)];
					projected_thumb_4 = inv(A_c) * [ projected_thumb_at_c(1) projected_thumb_at_c(2) projected_thumb_at_c(3) 1]';
					
					projected_thumb = [projected_thumb_4(x_axis) projected_thumb_4(y_axis) projected_thumb_4(z_axis)];
                    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The three points from A = hhand_q2cart.index_tip, B =
            % hhand_q2cart.little_tip, and C = projected_thumb consitute the
            % triangle to be mapped onto the bhand. We have to derive the
            % heigth, height base, center of gravity of the triangle ABC.
            % THESE ARE THE PARAMETERS TO USE IN BACKWARD SOLUTION:
            %
            corner_A = hhand_q2cart.index_tip;
            corner_B = hhand_q2cart.little_tip;
            corner_C = projected_thumb;
            
            corner_A_at_c_4     = A_c * [corner_A(1) corner_A(2) corner_A(3)   1]';
            corner_A_at_c       = [corner_A_at_c_4(x_axis)     corner_A_at_c_4(y_axis)   corner_A_at_c_4(z_axis)];
            
            corner_B_at_c_4     = A_c * [corner_B(1) corner_B(2) corner_B(3)   1]';
            corner_B_at_c       = [corner_B_at_c_4(x_axis)     corner_B_at_c_4(y_axis)   corner_B_at_c_4(z_axis)];

            world_origin_at_c_4 = A_c * [0 0 0 1]';
            world_origin_at_c   = [world_origin_at_c_4(x_axis)   world_origin_at_c_4(y_axis)   world_origin_at_c_4(z_axis)];
		
            % heigth of the mapping triangle
            height = sqrt(projected_thumb_at_c(1)^2 + projected_thumb_at_c(3)^2);
                                     
            heigth_base_length  = sqrt(  ( corner_A(x_axis) - corner_B(x_axis) )^2 +...
                                         ( corner_A(y_axis) - corner_B(y_axis) )^2 +...
                                         ( corner_A(z_axis) - corner_B(z_axis) )^2 );
                                     
            heigth_base_point   = c;
            
            center_of_gravity   = [ (corner_A(x_axis) + corner_B(x_axis) + corner_C(x_axis))/ 3,...
                                    (corner_A(y_axis) + corner_B(y_axis) + corner_C(y_axis))/ 3,...
                                    (corner_A(z_axis) + corner_B(z_axis) + corner_C(z_axis))/ 3];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        case strategy3          % use index-middle, ring-little pairs' and thumb's tips
            
        
            % Derive three points a,b,c on the digits' symmetry plane:
%              a = (hhand_q2cart.index_tip_past2 + hhand_q2cart.middle_tip_past2 + ...
%                   hhand_q2cart.ring_tip_past2 + hhand_q2cart.little_tip_past2)/ 4;
%                   
%              b = (hhand_q2cart.index_tip_past1 + hhand_q2cart.middle_tip_past1 + ...
%                   hhand_q2cart.ring_tip_past1 + hhand_q2cart.little_tip_past1)/ 4;
%               
             c = ( mapper.hhand.index_weigth  * hhand_q2cart.index_tip + ...
                   mapper.hhand.middle_weigth * hhand_q2cart.middle_tip + ...
                   mapper.hhand.ring_weigth   * hhand_q2cart.ring_tip + ...
                   mapper.hhand.little_weigth * hhand_q2cart.little_tip ) / 
                 ( mapper.hhand.index_weigth  + mapper.hhand.middle_weigth + ...
                   mapper.hhand.ring_weigth   + mapper.hhand.little_weigth);
                
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % NOW DERIVE THE SYMMETRIC FRAMES OF REFERENCE at Point c FOR BUILDING OF
                    % A SYMMETRY FRAME OF REFERENCE. 
                    %
                    % The four frames of references have little rotational 
                    c_at_index_tip = hhand.index.tip.A * [c(1) c(2) c(3) 1]';
					A_c1 = trans(c_at_index_tip(1), c_at_index_tip(2), c_at_index_tip(3)) * hhand.index.tip.A;

                    c_at_middle_tip = hhand.middle.tip.A * [c(1) c(2) c(3) 1]';
					A_c2 = trans(c_at_middle_tip(1), c_at_middle_tip(2), c_at_middle_tip(3)) * hhand.middle.tip.A;

					c_at_ring_tip = hhand.ring.tip.A * [c(1) c(2) c(3) 1]';
					A_c3 = trans(c_at_ring_tip(1), c_at_ring_tip(2), c_at_ring_tip(3)) * hhand.ring.tip.A;
                    
                    c_at_little_tip = hhand.little.tip.A * [c(1) c(2) c(3) 1]';
					A_c4 = trans(c_at_little_tip(1), c_at_little_tip(2), c_at_little_tip(3)) * hhand.little.tip.A;
                    
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					% Build a single frame of reference with a fusion of the two frames of
					% references. The idea is valid, because the fused frames of references are
					A_c = ( mapper.hhand.index_weigth  * A_c1 + ...
                            mapper.hhand.middle_weigth * A_c2 + ...
                            mapper.hhand.ring_weigth   * A_c3 + ...
                            mapper.hhand.little_weigth * A_c4 ) / ...
                          ( mapper.hhand.index_weigth + ...
                            mapper.hhand.middle_weigth + ...
                            mapper.hhand.ring_weigth + ...
                            mapper.hhand.little_weigth );
					
					thumb_tip_at_c = A_c * [hhand_q2cart.thumb_tip(1) hhand_q2cart.thumb_tip(2) hhand_q2cart.thumb_tip(3) 1]';
					
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					% Take the projection of the thumb tip on the x-z plane of A_c frame of
					% reference and transform t back to
                    
                    
					projected_thumb_at_c = [thumb_tip_at_c(x_axis)    0    thumb_tip_at_c(z_axis)];
					projected_thumb_4 = inv(A_c) * [ projected_thumb_at_c(1) projected_thumb_at_c(2) projected_thumb_at_c(3) 1]';
                    projected_thumb = [projected_thumb_4(x_axis) projected_thumb_4(y_axis) projected_thumb_4(z_axis)];
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % The three points from ABC = projected_thumb consitute the
                    % triangle to be mapped onto the bhand. We have to derive the
                    % heigth, height base, center of gravity of the triangle ABC.
                    %
                    % THESE ARE THE PARAMETERS TO USE IN BACKWARD SOLUTION:
                    %
                    corner_A = (hhand_q2cart.index_tip + hhand_q2cart.middle_tip)/ 2;
                    corner_B = (hhand_q2cart.ring_tip  + hhand_q2cart.little_tip)/ 2;
                    corner_C = projected_thumb;
                    
                    corner_A_at_c_4     = A_c * [corner_A(1) corner_A(2) corner_A(3)   1]';
                    corner_A_at_c       = [corner_A_at_c_4(x_axis)     corner_A_at_c_4(y_axis)   corner_A_at_c_4(z_axis)];
                    
                    corner_B_at_c_4     = A_c * [corner_B(1) corner_B(2) corner_B(3)   1]';
                    corner_B_at_c       = [corner_B_at_c_4(x_axis)     corner_B_at_c_4(y_axis)   corner_B_at_c_4(z_axis)];

                    world_origin_at_c_4 = A_c * [0 0 0 1]';
                    world_origin_at_c   = [world_origin_at_c_4(x_axis)   world_origin_at_c_4(y_axis)   world_origin_at_c_4(z_axis)];
				
                    heigth              = sqrt(  ( c(x_axis) - corner_C(x_axis) )^2 +...
                                                 ( c(y_axis) - corner_C(y_axis) )^2 +...
                                                 ( c(z_axis) - corner_C(z_axis) )^2); % heigth of the mapping triangle
                                             
                    heigth_base_length  = sqrt(  ( corner_A(x_axis) - corner_B(x_axis) )^2 +...
                                                 ( corner_A(y_axis) - corner_B(y_axis) )^2 +...
                                                 ( corner_A(z_axis) - corner_B(z_axis) )^2 );
                                             
                    heigth_base_point   = c;
                    
                    center_of_gravity   = [ (corner_A(x_axis) + corner_B(x_axis) + corner_C(x_axis))/ 3,...
                                            (corner_A(y_axis) + corner_B(y_axis) + corner_C(y_axis))/ 3,...
                                            (corner_A(z_axis) + corner_B(z_axis) + corner_C(z_axis))/ 3];
    stop = 1;                                    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
	end % end the switch
end     % end if(angle_index > 2)