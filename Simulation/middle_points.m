%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a procedure to calculate middle finger's link points in the link's own 
% frame of reference and to bring them back to the world frame of reference, so
% that all the points from all the fingers and joints canbe studied
% together.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    
% hhand_q2cart.middle => #columns : length_resolution * "number of links"
%                        #rows    : "number of axes"
%
hhand_q2cart.middle = zeros(length_resolution * 3, 3);

for length_index = 1:length_resolution
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % MIDDLE FINGER Cartesian points in world coordinates are:
     % proximal points
      hhand_q2cart.middle(length_index, :) = ...
          hom2vect( inv( trans(0, 0, hhand.middle.prox.A_delta_x(length_index))...
                                                     * hhand.middle.prox.A ));  
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                 

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % medial points:
      hhand_q2cart.middle(length_index + length_resolution, :) = ...
          hom2vect( inv( trans(0, 0, hhand.middle.media.A_delta_x(length_index))...
                                                     * hhand.middle.media.A ));        
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                 

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % distal points
      hhand_q2cart.middle(length_index + length_resolution * 2, :) = ...
          hom2vect( inv( trans(0, 0, hhand.middle.distal.A_delta_x(length_index))...
                                                     * hhand.middle.distal.A ));        
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
     
 end
 
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % tip point:
     if(with_memory)
        % First save the past tip point for trajectory building:
        hhand_q2cart.middle_tip_past2 = hhand_q2cart.middle_tip_past1;
        hhand_q2cart.middle_tip_past1 = hhand_q2cart.middle_tip;
     end
        
        % Now calcute the current tip point:     
      hhand_q2cart.middle_tip = hom2vect( inv(hhand.middle.tip.A)); 
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
