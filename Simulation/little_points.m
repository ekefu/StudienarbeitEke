%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a procedure to calculate little finger's link points in the link's own 
% frame of reference and to bring them back to the world frame of reference, so
% that all the points from all the fingers and joints canbe studied
% together.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  
% hhand_q2cart.little => #columns : length_resolution * "number of links"
%                        #rows    : "number of axes"
%
hhand_q2cart.little = zeros(length_resolution * 3, 3);

for length_index = 1:length_resolution
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % LITTLE FINGER Cartesian points in world coordinates are:
     % proximal points
      hhand_q2cart.little(length_index, :) = ...
          hom2vect( inv( trans(0, 0, hhand.little.prox.A_delta_x(length_index))...
                                                     * hhand.little.prox.A ));  
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                 

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % medial points:
      hhand_q2cart.little(length_index + length_resolution, :) = ...
          hom2vect( inv( trans(0, 0, hhand.little.media.A_delta_x(length_index))...
                                                     * hhand.little.media.A ));        
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                 

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % distal points
      hhand_q2cart.little(length_index + length_resolution * 2, :) = ...
          hom2vect( inv( trans(0, 0, hhand.little.distal.A_delta_x(length_index))...
                                                     * hhand.little.distal.A ));        
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
end


     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % tip point:
     if(with_memory)
        % First save the past tip point for trajectory building:
        hhand_q2cart.little_tip_past2 = hhand_q2cart.little_tip_past1; 
        hhand_q2cart.little_tip_past1 = hhand_q2cart.little_tip;
     end
        
        % Now calcute the current tip point:     
      hhand_q2cart.little_tip = hom2vect( inv(hhand.little.tip.A)); 
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
