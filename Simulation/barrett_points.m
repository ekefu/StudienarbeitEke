%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    BARRETT HAND link points in world coordinates (all fingers)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% hhand_q2cart.index => #columns : length_resolution * "number of links"
%                       #rows    : "number of axes"
%
bhand_q2cart.left = zeros(length_resolution * 3, 3);
bhand_q2cart.right = zeros(length_resolution * 3, 3);
bhand_q2cart.nonspread = zeros(length_resolution * 2, 3);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Palm points (computed new in each cycle because of palm alignment): 
       bpalm = [ hom2vect( inv( trans(0.0001, 0, 0) * barrett.left.base.A ))...
                 hom2vect( inv( trans(0.0001, 0, 0) * barrett.right.base.A ))...
                 hom2vect( inv( trans(0.0001, 0, 0) * barrett.nonspread.base.A ))... 
                 hom2vect( inv( trans(0.0001, 0, 0) * barrett.left.base.A ))  ];    
    %
    % Visit all the finger bases including the fpcorner, beginning and ending
    % in nonspread base. (nonspread, left, right, nonspread)
       bpalm_x = bpalm(x_axis,:);
       bpalm_y = bpalm(y_axis,:);
       bpalm_z = bpalm(z_axis,:);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       

    
for length_index = 1:length_resolution
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % LEFT FINGER Cartesian points in world coordinates are:
     %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % proximal points:
       bhand_q2cart.left(length_index, :) = ...
            hom2vect( inv(  trans(barrett.left.abduct.A_delta_x(length_index), 0, 0)...
                                                            * barrett.left.abduct.A ));
     % medial points:                                                        
       bhand_q2cart.left(length_index + length_resolution, :) = ...
            hom2vect( inv(  trans(barrett.left.media.A_delta_x(length_index), 0, 0)...   
                                                            * barrett.left.media.A ));        
     % distal points:                                                        
       bhand_q2cart.left(length_index + length_resolution * 2, :) = ...                                                        
            hom2vect( inv(  trans(barrett.left.distal.A_delta_x(length_index), 0, 0)...  
                                                            * barrett.left.distal.A ));    
     % tip point:                                                        
       bhand_q2cart.left_tip = hom2vect( inv(barrett.left.tip.A ));    
         
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                  
                                  
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % RIGHT FINGER Cartesian points in world coordinates are:
     %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % proximal points:
       bhand_q2cart.right(length_index, :) = ...
            hom2vect( inv(  trans(barrett.right.abduct.A_delta_x(length_index), 0, 0)...
                                                            * barrett.right.abduct.A ));        
     % medial points:                                                        
       bhand_q2cart.right(length_index + length_resolution, :) = ...
            hom2vect( inv(  trans(barrett.right.media.A_delta_x(length_index), 0, 0)...   
                                                            * barrett.right.media.A ));        
     % distal points:                                                        
       bhand_q2cart.right(length_index + length_resolution * 2, :) = ...                                                        
            hom2vect( inv(  trans(barrett.right.distal.A_delta_x(length_index), 0, 0)...  
                                                            * barrett.right.distal.A ));        
     % tip point:                                                        
       bhand_q2cart.right_tip = hom2vect( inv(barrett.right.tip.A ));    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                  
                                  
                                  
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % NONSPREAD FINGER Cartesian points in world coordinates are:
     %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % proximal points:
       bhand_q2cart.nonspread(length_index, :) = ...
            hom2vect( inv(  trans(barrett.nonspread.media.A_delta_x(length_index), 0, 0)...
                                                            * barrett.nonspread.media.A ));        
     % medial points:                                                        
       bhand_q2cart.nonspread(length_index + length_resolution, :) = ...
            hom2vect( inv(  trans(barrett.nonspread.distal.A_delta_x(length_index), 0, 0)...   
                                                            * barrett.nonspread.distal.A ));         
     % tip point:                                                        
       bhand_q2cart.nonspread_tip = hom2vect( inv(barrett.nonspread.tip.A ));    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end