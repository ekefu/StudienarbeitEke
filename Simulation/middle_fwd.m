%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% For delta positions: the location RELATIVE TO THE base frame will be as
% follows:
%   point_Adelta_Link21 = hhand.middle.prox.A * [hhand.index.prox.delta_x(i); 0; 0; 1]   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% =>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proximal link base frame for delta points 

hhand.middle.prox.A   = rotate(y_axis, pi/2 + hhand.middle.prox.rt_Q)...      
                      * rotate(z_axis, hhand.middle.abduct.rt_Q)...
                      * hhand.middle.base.A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Medial link base frame for delta positions 
hhand.middle.media.A  = rotate(y_axis, hhand.middle.media.rt_Q)...  
                      * trans(0, 0, hhand.middle.prox.length)...
                      * hhand.middle.prox.A;
                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distal link base frame for delta positions
hhand.middle.distal.A = rotate(y_axis, hhand.middle.distal.rt_Q)...
                      * trans(0, 0, hhand.middle.media.length)...
                      * hhand.middle.media.A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finger tip base frame:
hhand.middle.tip.A     = trans(0, 0, hhand.middle.distal.length)... 
                      * hhand.middle.distal.A;
                  
                  