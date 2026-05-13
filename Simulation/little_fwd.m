%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% For delta positions: the location RELATIVE TO THE base frame will be as
% follows:
%   point_Adelta_Link21 = hhand.little.prox.A * [hhand.index.prox.delta_x(i); 0; 0; 1]   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% =>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proximal link base frame for delta points 

hhand.little.prox.A   = rotate(y_axis, pi/2 + hhand.little.prox.rt_Q)...      
                      * rotate(z_axis, hhand.little.abduct.rt_Q)...
                      * hhand.little.base.A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Medial link base frame for delta positions 
hhand.little.media.A  = rotate(y_axis, hhand.little.media.rt_Q)...  
                      * trans(0, 0, hhand.little.prox.length)...
                      * hhand.little.prox.A;
                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distal link base frame for delta positions
hhand.little.distal.A = rotate(y_axis, hhand.little.distal.rt_Q)...
                      * trans(0, 0, hhand.little.media.length)...
                      * hhand.little.media.A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finger tip base frame:
hhand.little.tip.A    = trans(0, 0, hhand.little.distal.length)... 
                      * hhand.little.distal.A;
                  
                    