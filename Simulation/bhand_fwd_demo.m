%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This demo simulates and visualises a closing movement of the bhand 
% fingers. 

% Now set up bhand:
mode_b              = close_sim_mode;

% in "bhand_fwd_demo" the origin-shifter is "off"
mapper.barrett.rotate_z    = 0;
mapper.barrett.rotate_y    = 0; 
mapper.barrett.rotate_x    = 0;

mapper.barrett.trans_x     = 0;
mapper.barrett.trans_y     = 0;
mapper.barrett.trans_z     = 0;

mapper.barrett.scaler      = 1; % 1 means no scaling

% load bhand objects:
barrett_hand;

% Now start the bhand model derived with the fwd kinematics of bhand:
for angle_index = 1:angle_resolution_b   % angle_resolution_b = 128@closesim_mode

    bhand_control;

    plot3(...
               bpalm_x, bpalm_y, bpalm_z, 'r',...
               bhand_q2cart.left(:,x_axis),      bhand_q2cart.left(:,y_axis),      bhand_q2cart.left(:,z_axis),...
               bhand_q2cart.right(:,x_axis),     bhand_q2cart.right(:,y_axis),     bhand_q2cart.right(:,z_axis),...
               bhand_q2cart.nonspread(:,x_axis), bhand_q2cart.nonspread(:,y_axis), bhand_q2cart.nonspread(:,z_axis))
        grid on
        axis([-100 100 -100 100 -100 100])
        xlabel('dimensions in mm')
    
    F(angle_index) = getframe(gcf);
end

%movie2avi(movie(F), 'bhand_fwd_demo.avi');