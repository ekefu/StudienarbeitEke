%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a procedure to calculate index finger's link points in the link's own 
% frame of reference and to bring them back to the world frame of reference, so
% that all the points from all the fingers and joints canbe studied
% together.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    
% hhand_q2cart.index => #columns : length_resolution * "number of links"
%                       #rows    : "number of axes"
%
hhand_q2cart.index = zeros(length_resolution * 3, 3);

for length_index = 1:length_resolution
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % INDEX FINGER Cartesian points in world coordinates are:
     %
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % proximal points:
      hhand_q2cart.index(length_index, :) = ...
          hom2vect( inv( trans(0, 0, hhand.index.prox.A_delta_x(length_index))...
                                                     * hhand.index.prox.A ));   
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                     
                                                 
     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % medial points:
      hhand_q2cart.index(length_index + length_resolution, :) = ...
          hom2vect( inv( trans(0, 0, hhand.index.media.A_delta_x(length_index))...
                                                     * hhand.index.media.A ));  
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                     

     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % distal points:
      hhand_q2cart.index(length_index + length_resolution * 2, :) = ...
          hom2vect( inv( trans(0, 0, hhand.index.distal.A_delta_x(length_index))...
                                                     * hhand.index.distal.A )); 
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
     
end


     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % tip point:
     if(with_memory)
        % First save the past tip point for trajectory building:
        hhand_q2cart.index_tip_past2 = hhand_q2cart.index_tip_past1; 
        hhand_q2cart.index_tip_past1 = hhand_q2cart.index_tip; 
     end
        
        % Now calcute the current tip point:     
      hhand_q2cart.index_tip = hom2vect( inv(hhand.index.tip.A)); 
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
