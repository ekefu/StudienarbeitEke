        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% use left-right, ring-little pairs' and nonspread's tips
    
    % Derive central point on the digits' symmetry plane:

     bspread_center = ( bhand_q2cart.left_tip + ...
                        bhand_q2cart.right_tip ) / 3;
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % NOW DERIVE THE SYMMETRIC FRAMES OF REFERENCE at Point c FOR BUILDING OF
    % A SYMMETRY FRAME OF REFERENCE. 
    %
    % The four frames of references have little rotational differences; in
    % order to find the orientation of the symmetry frame of reference, the
    % fingertip frames should be shifted to the spread center and fused
    % together:
    
    % From left:
    bspread_center_at_left_tip  = bhand.left.tip.A * [ spread_center(x_axis)...
                                                       spread_center(y_axis)...
                                                       spread_center(z_axis) 1]';
	A_bspread_center1 = trans( spread_center_at_left_tip(x_axis),...
                               spread_center_at_left_tip(y_axis),...
                               spread_center_at_left_tip(z_axis) ) * bhand.left.tip.A;

    % From right:                          
    spread_center_at_right_tip = bhand.right.tip.A * [ spread_center(x_axis)...
                                                       spread_center(y_axis)...
                                                       spread_center(z_axis) 1]';
	A_bspread_center2 = trans( spread_center_at_right_tip(x_axis),...
                               spread_center_at_right_tip(y_axis),...
                               spread_center_at_right_tip(z_axis) ) * bhand.right.tip.A;
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Build a single frame of reference with a fusion of the two frames of
	% references. The idea is valid, because the fused frames of references are
	A_bsym_frame = ( A_bspread_center1 + A_bspread_center2 ) / 2;
	
	nonspread_tip_at_sym_frame = A_sym_frame * [ bhand_q2cart.nonspread_tip(x_axis)...
                                                 bhand_q2cart.nonspread_tip(y_axis)...
                                                 bhand_q2cart.nonspread_tip(z_axis) 1]';
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Take the projection of the nonspread tip on the x-z plane of A_sym_frame 
    % frame of reference and transform t back to
    
	projected_nonspread_at_sym_frame = [nonspread_tip_at_sym_frame(x_axis)    0    nonspread_tip_at_sym_frame(z_axis)];
	projected_nonspread_4            = inv(A_sym_frame) * [ projected_nonspread_at_sym_frame(x_axis)...
                                                            projected_nonspread_at_sym_frame(y_axis)... 
                                                            projected_nonspread_at_sym_frame(z_axis) 1]';
                                        
    projected_nonspread_at_world     = [projected_nonspread_4(x_axis) projected_nonspread_4(y_axis) projected_nonspread_4(z_axis)];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The three points from ABC = projected_nonspread consitute the
    % triangle to be mapped onto the bhand. We have to derive the
    % heigth, height base, center of gravity of the triangle ABC.
    %
    % THESE ARE THE PARAMETERS TO USE IN BACKWARD SOLUTION:
    %
    corner_A = bhand_q2cart.left_tip;
           
    corner_B = bhand_q2cart.right_tip;
    
    corner_C = projected_nonspread_at_world;
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
        corner_A_at_sym_frame_4     = A_bsym_frame * [ corner_A(x_axis) corner_A(y_axis) corner_A(z_axis)   1]';
        corner_A_at_sym_frame       = [ corner_A_at_sym_frame_4(x_axis)...
                                        corner_A_at_sym_frame_4(y_axis)...
                                        corner_A_at_sym_frame_4(z_axis)];
        
        corner_B_at_sym_frame_4     = A_sym_frame * [ corner_B(x_axis) corner_B(y_axis) corner_B(z_axis)   1]';
        corner_B_at_sym_frame       = [ corner_B_at_sym_frame_4(x_axis)...
                                        corner_B_at_sym_frame_4(y_axis)...
                                        corner_B_at_sym_frame_4(z_axis) ];
                                
        corner_C_at_sym_frame       = projected_nonspread_at_sym_frame;                                

        world_origin_at_sym_frame_4 = A_sym_frame * [0 0 0 1]';
        world_origin_at_sym_frame   = [ world_origin_at_sym_frame_4(x_axis)...
                                        world_origin_at_sym_frame_4(y_axis)...
                                        world_origin_at_sym_frame_4(z_axis)];
     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	

stop = 1;                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%