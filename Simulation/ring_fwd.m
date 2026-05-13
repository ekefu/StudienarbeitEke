%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% For delta positions: the location RELATIVE TO THE base frame will be as
% follows:
%   point_Adelta_Link21 = hhand.ring.prox.A * [hhand.index.prox.delta_x(i); 0; 0; 1]   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% =>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proximal link base frame for delta points 

hhand.ring.prox.A    = rotate(y_axis, pi/2 + hhand.ring.prox.rt_Q)...      
                      * rotate(z_axis, hhand.ring.abduct.rt_Q)...
                      * hhand.ring.base.A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Medial link base frame for delta positions 
hhand.ring.media.A   = rotate(y_axis, hhand.ring.media.rt_Q)...  
                      * trans(0, 0, hhand.ring.prox.length)...
                      * hhand.ring.prox.A;
                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distal link base frame for delta positions
hhand.ring.distal.A  = rotate(y_axis, hhand.ring.distal.rt_Q)...
                      * trans(0, 0, hhand.ring.media.length)...
                      * hhand.ring.media.A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finger tip base frame:
hhand.ring.tip.A     = trans(0, 0, hhand.ring.distal.length)... 
                      * hhand.ring.distal.A;
                  
                  