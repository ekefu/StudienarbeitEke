%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For delta positions: the location RELATIVE TO THE base frame will be as
% follows:
%   point_Adelta_Link21 = hhand.index.prox.A * [hhand.index.prox.delta_x(i); 0; 0; 1]   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% =>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proximal link base frame for delta points 
hhand.index.prox.A    = rotate(y_axis, pi/2 + hhand.index.prox.rt_Q)...      
                      * rotate(z_axis, hhand.index.abduct.rt_Q)...
                      * hhand.index.base.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Medial link base frame for delta positions 
hhand.index.media.A   = rotate(y_axis, hhand.index.media.rt_Q)...  
                      * trans(0, 0, hhand.index.prox.length)...
                      * hhand.index.prox.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  
                  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distal link base frame for delta positions
hhand.index.distal.A  = rotate(y_axis, hhand.index.distal.rt_Q)...
                      * trans(0, 0, hhand.index.media.length)...
                      * hhand.index.media.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finger tip base frame:
hhand.index.tip.A     = trans(0, 0, hhand.index.distal.length)... 
                      * hhand.index.distal.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  
                  