%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% THIS IS THE MAIN CONTROL LOOP. THE FOLLOWING STEPS ARE IMPLEMENTED: 
%
% 1) DERIVATION OF A SAMPLE JOINTSPACE FOR HHAND FROM A TESTBENCH SCENARIO;
% THE FORWARD SOLUTION
% 2) DERIVATION OF DESIRED VALUES FOR BHAND CONTROL IN CARTESIAN SPACE;
% THE MAPPING
% 3) DERIVATION OF THE NECESSARY CONTROL VECTOR OR THE DESIRED VALUES;
% THE BACKWARD SOLUTION
% 4) VISUALISATION OF THE CURRENT HHAND AND BHAND POSTURES.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

save_demo_to_avi = 0;

if(save_demo_to_avi)
    mov = avifile('barrett_hand.avi', 'compression', 'none', 'fps', 1);
end

if(mapper_optimise_mode == 0)
    mapper.barrett.rotate_z    = 0;
	mapper.barrett.rotate_y    = 0; 
	mapper.barrett.rotate_x    = 0;
	
	mapper.barrett.trans_x     = 0;
	mapper.barrett.trans_y     = 0;
	mapper.barrett.trans_z     = 0;
	mapper.barrett.scaler      = 1;
	
	mapper.hhand.index_weight  = 1;
	mapper.hhand.middle_weight = 1;
	mapper.hhand.ring_weight   = 1;
	mapper.hhand.little_weight = 1;
end
    
    % Now set up hhand:
    mode_h              = synergic_close; 
  % mode_h              = single_index_close;
  
    % now load hhand objects with the above setting
    human_hand;
            
    mode_b              = control_mode;
    % load bhand objects:
    barrett_hand;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

% "demo_map_count" sets how long the mapping function will be demonstrated
demo_map_count = 40;

loop_fitness = 0;

% Start mapping loop operation:
for steps = 1:demo_map_count
    % first generate the actual hand positions
    hhand_control;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now map the current hhand configuration onto the bhand:
    %bhand_mapping_and_rwl;
     mapper_5_to_3;
     
     bhand_rwl;
     
    % Derive the bhand points for the derived bhand joint vector:
     bhand_control;        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%    CONTINUOUS ADAPTATION OF ORIGIN-SHIFT CAN ALSO BE PUT INTO THE LOOP
	%    HERE; WE WILL ASSUME THAT, A SIMILAR LOOP HAS ALREADY OPTIMISED THE 
	%    ORIGIN-SHIFT PARAMETERS!
	if(mapper_optimise_mode)
       % derive the fitness function
	
	    single_fit =( (corner_A(x_axis) - bhand_q2cart.left_tip(x_axis) )^2 + ...
                      (corner_A(y_axis) - bhand_q2cart.left_tip(y_axis) )^2 + ...
                      (corner_A(z_axis) - bhand_q2cart.left_tip(z_axis) )^2 + ...
                      (corner_B(x_axis) - bhand_q2cart.right_tip(x_axis) )^2 + ...
                      (corner_B(y_axis) - bhand_q2cart.right_tip(y_axis) )^2 + ...
                      (corner_B(z_axis) - bhand_q2cart.right_tip(z_axis) )^2 + ...
                      (corner_C(x_axis) - bhand_q2cart.nonspread_tip(x_axis) )^2 + ...
                      (corner_C(y_axis) - bhand_q2cart.nonspread_tip(y_axis) )^2 + ...
                      (corner_C(z_axis) - bhand_q2cart.nonspread_tip(z_axis) )^2  ) / 9;
                  
        % calculate the cumulative fitness for all postures          
        loop_fitness = loop_fitness + single_fit;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  Now plot the resulting mapping:
    subplot(1,2,1),...
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
    

    subplot(1,2,2),plot3(...
           bpalm_x, bpalm_y, bpalm_z, 'r',...
           bhand_q2cart.left(:,x_axis),      bhand_q2cart.left(:,y_axis),      bhand_q2cart.left(:,z_axis),...
           bhand_q2cart.right(:,x_axis),     bhand_q2cart.right(:,y_axis),     bhand_q2cart.right(:,z_axis),...
           bhand_q2cart.nonspread(:,x_axis), bhand_q2cart.nonspread(:,y_axis), bhand_q2cart.nonspread(:,z_axis))
    grid on
    axis([-100 100 -100 100 -100 100])
            ylabel('dimensions in mm')

    %F = getframe;
    
     F = getframe(gcf);
    
    if(save_demo_to_avi)
        mov = addframe(mov,F);
        clf
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end  

if(save_demo_to_avi)
    mov = close(mov);
end