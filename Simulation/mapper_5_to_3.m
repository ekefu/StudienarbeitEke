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
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% use index-middle, ring-little pairs' and thumb's tips
    
    % Derive central point on the digits' symmetry plane:

     spread_center = (  mapper.hhand.index_weight  * hhand_q2cart.index_tip + ...
                        mapper.hhand.middle_weight * hhand_q2cart.middle_tip + ...
                        mapper.hhand.ring_weight   * hhand_q2cart.ring_tip + ...
                        mapper.hhand.little_weight * hhand_q2cart.little_tip ) / ... 
                      ( mapper.hhand.index_weight  + mapper.hhand.middle_weight + ...
                        mapper.hhand.ring_weight   + mapper.hhand.little_weight);
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % NOW DERIVE THE SYMMETRIC FRAMES OF REFERENCE at Point c FOR BUILDING OF
    % A SYMMETRY FRAME OF REFERENCE. 
    %
    % The four frames of references have little rotational differences; in
    % order to find the orientation of the symmetry frame of reference, the
    % fingertip frames should be shifted to the spread center and fused
    % together:
    
    % From index:
    spread_center_at_index_tip  = hhand.index.tip.A * [ spread_center(x_axis)...
                                                        spread_center(y_axis)...
                                                        spread_center(z_axis) 1]';
	A_spread_center1 = trans( spread_center_at_index_tip(x_axis),...
                              spread_center_at_index_tip(y_axis),...
                              spread_center_at_index_tip(z_axis) ) * hhand.index.tip.A;

    % From middle:                          
    spread_center_at_middle_tip = hhand.middle.tip.A * [ spread_center(x_axis)...
                                                         spread_center(y_axis)...
                                                         spread_center(z_axis) 1]';
	A_spread_center2 = trans( spread_center_at_middle_tip(x_axis),...
                              spread_center_at_middle_tip(y_axis),...
                              spread_center_at_middle_tip(z_axis) ) * hhand.middle.tip.A;
    
    % From ring:                      
    spread_center_at_ring_tip   = hhand.ring.tip.A * [ spread_center(x_axis)...
                                                       spread_center(y_axis)...
                                                       spread_center(z_axis) 1]';
	A_spread_center3 = trans( spread_center_at_ring_tip(x_axis),...
                              spread_center_at_ring_tip(y_axis),...
                              spread_center_at_ring_tip(z_axis) ) * hhand.ring.tip.A;
    
    % From little:                      
    spread_center_at_little_tip = hhand.little.tip.A * [ spread_center(x_axis)...
                                                         spread_center(y_axis)...
                                                         spread_center(z_axis) 1]';
	A_spread_center4 = trans( spread_center_at_little_tip(x_axis),...
                              spread_center_at_little_tip(y_axis),...
                              spread_center_at_little_tip(z_axis) ) * hhand.little.tip.A;
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Build a single frame of reference with a fusion of the two frames of
	% references. The idea is valid, because the fused frames of references are
	A_sym_frame = ( mapper.hhand.index_weight  * A_spread_center1 + ...
                    mapper.hhand.middle_weight * A_spread_center2 + ...
                    mapper.hhand.ring_weight   * A_spread_center3 + ...
                    mapper.hhand.little_weight * A_spread_center4 ) / ...
                  ( mapper.hhand.index_weight + ...
                    mapper.hhand.middle_weight + ...
                    mapper.hhand.ring_weight + ...
                    mapper.hhand.little_weight );
	
	thumb_tip_at_sym_frame = A_sym_frame * [ hhand_q2cart.thumb_tip(x_axis)...
                                             hhand_q2cart.thumb_tip(y_axis)...
                                             hhand_q2cart.thumb_tip(z_axis) 1]';
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Take the projection of the thumb tip on the x-z plane of A_sym_frame 
    % frame of reference and transform t back to
    
	projected_thumb_at_sym_frame = [thumb_tip_at_sym_frame(x_axis)    0    thumb_tip_at_sym_frame(z_axis)];
	projected_thumb_4            = inv(A_sym_frame) * [ projected_thumb_at_sym_frame(x_axis)...
                                                        projected_thumb_at_sym_frame(y_axis)... 
                                                        projected_thumb_at_sym_frame(z_axis) 1]';
                                        
    projected_thumb_at_world     = [projected_thumb_4(x_axis) projected_thumb_4(y_axis) projected_thumb_4(z_axis)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The three points from ABC = projected_thumb consitute the
    % triangle to be mapped onto the bhand. We have to derive the
    % heigth, height base, center of gravity of the triangle ABC.
    %
    % THESE ARE THE PARAMETERS TO USE IN BACKWARD SOLUTION:
    %
    corner_A = ( mapper.hhand.index_weight  * hhand_q2cart.index_tip + ...
                 mapper.hhand.middle_weight * hhand_q2cart.middle_tip) / ...
               ( mapper.hhand.index_weight  + mapper.hhand.middle_weight );
           
    corner_B = ( mapper.hhand.ring_weight   * hhand_q2cart.ring_tip  + ...
                 mapper.hhand.little_weight * hhand_q2cart.little_tip) / ...
               ( mapper.hhand.ring_weight   + mapper.hhand.little_weight );
    
    corner_C = projected_thumb_at_world;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % if the mapping vector includes the dimensions of the finger tips
    % triangle, they have to be calculated here:
    if(mode_j == triangle_dimensions)
        % heigth of the mapping triangle                        
        heigth              = sqrt( ( spread_center(x_axis) - corner_C(x_axis) )^2 +...
                                    ( spread_center(y_axis) - corner_C(y_axis) )^2 +...
                                    ( spread_center(z_axis) - corner_C(z_axis) )^2); 
                                 
        heigth_base_length  = sqrt(  ( corner_A(x_axis) - corner_B(x_axis) )^2 +...
                                     ( corner_A(y_axis) - corner_B(y_axis) )^2 +...
                                     ( corner_A(z_axis) - corner_B(z_axis) )^2 );
                                 
        heigth_base_point   = spread_center;
        
        center_of_gravity   = [ (corner_A(x_axis) + corner_B(x_axis) + corner_C(x_axis))/ 3,...
                                (corner_A(y_axis) + corner_B(y_axis) + corner_C(y_axis))/ 3,...
                                (corner_A(z_axis) + corner_B(z_axis) + corner_C(z_axis))/ 3];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % if the mapping vector includes endpoint coordinates at symmetry frame, 
    % they have to be calculated here:
    if(mode_j == symmetry_frame_coordinates)
        corner_A_at_sym_frame_4     = A_sym_frame * [ corner_A(x_axis) corner_A(y_axis) corner_A(z_axis)   1]';
        corner_A_at_sym_frame       = [ corner_A_at_sym_frame_4(x_axis)...
                                        corner_A_at_sym_frame_4(y_axis)...
                                        corner_A_at_sym_frame_4(z_axis)];
        
        corner_B_at_sym_frame_4     = A_sym_frame * [ corner_B(x_axis) corner_B(y_axis) corner_B(z_axis)   1]';
        corner_B_at_sym_frame       = [ corner_B_at_sym_frame_4(x_axis)...
                                        corner_B_at_sym_frame_4(y_axis)...
                                        corner_B_at_sym_frame_4(z_axis) ];
                                
        corner_C_at_sym_frame       = projected_thumb_at_sym_frame;                                

        world_origin_at_sym_frame_4 = A_sym_frame * [0 0 0 1]';
        world_origin_at_sym_frame   = [ world_origin_at_sym_frame_4(x_axis)...
                                        world_origin_at_sym_frame_4(y_axis)...
                                        world_origin_at_sym_frame_4(z_axis)];
     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	

stop = 1;                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
if(mode_j == corner_coordinates)
    simnet_input_vector = [ corner_A(1);   corner_A(2);   corner_A(3);...
                            corner_B(1);   corner_B(2);   corner_B(3);...
                            corner_C(1);   corner_C(2);   corner_C(3)];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
if(mode_j == triangle_dimensions)
    simnet_input_vector = [ corner_A(1);   corner_A(2);   corner_A(3);...
                            corner_B(1);   corner_B(2);   corner_B(3);...
                            corner_C(1);   corner_C(2);   corner_C(3)];    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
if(mode_j == symmetry_frame_coordinates)
    simnet_input_vector = [ corner_A_at_sym_frame(x_axis);...
                            corner_A_at_sym_frame(y_axis);...
                            corner_A_at_sym_frame(z_axis);...
                            corner_B_at_sym_frame(x_axis);...
                            corner_B_at_sym_frame(y_axis);...
                            corner_B_at_sym_frame(z_axis);...
                            corner_C_at_sym_frame(x_axis);...
                            corner_C_at_sym_frame(y_axis);...
                            corner_C_at_sym_frame(z_axis) ];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        