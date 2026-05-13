%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This demo simulates and visualises a closing movement of the hhand 
% fingers. 

% First set up bhand:
mode_h              = synergic_close;

% Now load hhand objects with the above setting
 human_hand;


% Now start the bhand model derived with the fwd kinematics of bhand:
for demo_index = 1:120  

    hhand_control;

    plot3(...
          palm_x, palm_y, palm_z, 'r',...
          hhand_q2cart.index(:, x_axis) , hhand_q2cart.index(:,y_axis),  hhand_q2cart.index(:,z_axis),...
          hhand_q2cart.middle(:, x_axis), hhand_q2cart.middle(:,y_axis), hhand_q2cart.middle(:,z_axis),...
          hhand_q2cart.ring(:, x_axis),   hhand_q2cart.ring(:,y_axis),   hhand_q2cart.ring(:,z_axis),...
          hhand_q2cart.little(:, x_axis), hhand_q2cart.little(:,y_axis), hhand_q2cart.little(:,z_axis),...
          hhand_q2cart.thumb(:, x_axis),  hhand_q2cart.thumb(:,y_axis),  hhand_q2cart.thumb(:,z_axis))
    grid on
%           thumb_x_of_m(angle_index), 0, 0, '+',...
%           0, 0, thumb_z_of_m(angle_index), 'x',...
    axis([-100 100 -100 100 -100 100])
            ylabel('dimensions in mm')
    
    F(angle_index) = getframe(gcf);
end

%movie2avi(movie(F), 'bhand_fwd_demo.avi');