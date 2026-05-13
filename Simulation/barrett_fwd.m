%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Base points for the two spread fingers is constant:
if (mode_b == control_mode)
    % Alignment transformation is adapted with mutation-selection
    % optimisation. Transformation uses 3D shift and raw, pitch, yaw
    % orientation. See "mapper_optmisation.m".
    barrett.alignment.A    = rotate(z_axis, mapper.barrett.rotate_z)...
                           * rotate(y_axis, mapper.barrett.rotate_y)...
                           * rotate(x_axis, mapper.barrett.rotate_x)... 
                           * trans(  mapper.barrett.trans_x,...
                                     mapper.barrett.trans_y,...
                                     mapper.barrett.trans_z );
    % now generate the scaling matrix                                       
    scaling_matrix         = mapper.barrett.scaler * eye(4);
    scaling_matrix(4,4)    = 1;
    
    % Now add the scaling effect onto the alignment matrix
    barrett.alignment.A    = barrett.alignment.A * scaling_matrix;
else                                       
    barrett.alignment.A    = eye(4); 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Base points for the two spread fingers is constant:
barrett.left.base.A        = trans(-20,-25,0) * barrett.alignment.A; 
barrett.right.base.A       = trans(-20,25,0)  * barrett.alignment.A; 
barrett.nonspread.base.A   = trans(30,0,0)    * barrett.alignment.A; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The proximal link for the two spread fingers has only abduction movement. 
% Delta points (points on the proximal link: 0 - 50mm)
barrett.left.abduct.A      = rotate(z_axis,  barrett.left.abduct.rt_Q)...
                           * barrett.left.base.A;   % pi/2 offset?
            
barrett.right.abduct.A     = rotate(z_axis, -barrett.right.abduct.rt_Q)... 
                           * barrett.right.base.A;  % pi/2 offset?
% There is no proximal freedom which undergoes abduction or flexion for the
% nonspread finger.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Medial link delta positions (points on the proximal link: 0 - 70mm)
barrett.left.media.A       = rotate(y_axis, -barrett.left.media.rt_Q)...
                           * trans(barrett.left.abduct.length, 0, 0)...
                           * barrett.left.abduct.A;

barrett.right.media.A      = rotate(y_axis, -barrett.right.media.rt_Q)...
                           * trans(barrett.right.abduct.length, 0, 0)... 
                           * barrett.right.abduct.A;

barrett.nonspread.media.A  = rotate(y_axis, -barrett.nonspread.media.rt_Q)...
                           * barrett.nonspread.base.A;
% Since there is no proximal freedom for the nonspread finger;
% barrett.nonspread.prox.A == barrett.nonspread.base.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distal link delta positions (points on the proximal link: 0 - 56mm)
%barrett.left.distal.A      = rotate(y_axis, -pi/4 - barrett.left.distal.rt_Q)...
barrett.left.distal.A      = rotate(y_axis, -barrett.left.distal.rt_Q)...
                           * trans(barrett.left.media.length, 0, 0)...
                           * barrett.left.media.A;

%barrett.right.distal.A     = rotate(y_axis, -pi/4 - barrett.right.distal.rt_Q)...
barrett.right.distal.A     = rotate(y_axis, -barrett.right.distal.rt_Q)...
                           * trans(barrett.right.media.length, 0, 0)...
                           * barrett.right.media.A;

%barrett.nonspread.distal.A = rotate(y_axis, -pi/4 - barrett.nonspread.distal.rt_Q)...
barrett.nonspread.distal.A = rotate(y_axis, -barrett.nonspread.distal.rt_Q)...
                           * trans(barrett.nonspread.media.length, 0, 0)...
                           * barrett.nonspread.media.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
%Finger Tip frames:
barrett.left.tip.A         = trans(barrett.left.distal.length, 0, 0)...
                           * barrett.left.distal.A;
          
barrett.right.tip.A        = trans(barrett.right.distal.length, 0, 0)...
                           * barrett.right.distal.A;
          
barrett.nonspread.tip.A    = trans(barrett.nonspread.distal.length, 0, 0)...
                           * barrett.nonspread.distal.A;          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                          