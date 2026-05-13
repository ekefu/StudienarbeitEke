%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% For delta positions: the location RELATIVE TO THE base frame will be as
% follows:
%   point_Adelta_Link21 = hhand.thumb.prox.A * [hhand.index.prox.delta_x(i); 0; 0; 1]   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% =>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proximal link base frame for delta points 
% Thumb base frame will be rotated with an offset, the joint
% movements will be added onto the offset rotation. The offsets are to be calibrated.
% -25° = -((pi*5)/36)  &  -135° = -((pi*3)/4)
hhand.thumb.prox.A    = rotate(x_axis, hhand.thumb.prox.rt_Q + pi/18 )...          
                      * rotate(z_axis, hhand.thumb.abduct.rt_Q - 3*pi/4 )...
                      * hhand.thumb.base.A;   % rotate z oldu, sorun digerinde..
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Medial link base frame for delta positions 
hhand.thumb.media.A   = rotate(z_axis, hhand.thumb.media.rt_Q)...
                      * trans(0, hhand.thumb.prox.length, 0)...  
                      * hhand.thumb.prox.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  
                  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distal link base frame for delta positions
hhand.thumb.distal.A  = rotate(z_axis, hhand.thumb.distal.rt_Q)...
                      * trans(0, hhand.thumb.media.length, 0)...
                      * hhand.thumb.media.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Finger tip base frame:
hhand.thumb.tip.A     = trans(0, hhand.thumb.distal.length, 0)... 
                      * hhand.thumb.distal.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  
                  
                  
                  