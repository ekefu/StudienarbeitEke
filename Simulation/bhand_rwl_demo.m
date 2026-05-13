%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bhand backward kinematics numerical (network) solution demo:
%        "bhand_rwl_demo.m"
%
% (call initials_for_robotics with necessary mode choice)
% 
% After bhand finger distal link reaches its limit angle pi/2, the link
% does not move any more. After this border, the two link behaves
% like a single body and the cartesian trajectory on the flexion plane 
% becomes a circular arc. With the symmetric spread motion, all possible
% movements of the two spread fingers occur on a torus-like surface slice.
% Analytical equations expressing this workspace can be expressed in 
% spherical coordinates, but however, the variables from such equation 
% systems are not seperable into simple ODEs. Therefore, a numerical 
% training was carried out to find the backward kinematics solution.
%
% To be able to run the demo, a previously trained network should be
% existing for loading.
%
% The demo traces the training workspace randomly and the representation of
% the training set with the trained network can be visualised. Important
% point is, that the demonstration does not show the generalisation ability
% of the trained network, but 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Strategy for jacobian derivation
% triangle_dimensions = 1;
% corner_coordinates  = 2;
% 
% mode_j = corner_coordinates;
% 
% if(mode_j == triangle_dimensions)
%     bhand_rwl_net = 'rwl_triangle_elman';
% end
% if(mode_j == corner_coordinates)
%     bhand_rwl_net = 'rwl_points_elman';
% end

        mode_b              = control_mode;
        
        % in "bhand_rwl_demo", the origin_shifter is "off":
        mapper.barrett.rotate_z    = 0;
        mapper.barrett.rotate_y    = 0; 
        mapper.barrett.rotate_x    = 0;

        mapper.barrett.trans_x     = 0;
        mapper.barrett.trans_y     = 0;
        mapper.barrett.trans_z     = 0;
        mapper.barrett.scaler      = 1; % 1 means no scaling

        % load barrett hand objects:
        barrett_hand;
        

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now start genarating training set I/O pairs, which will visit possible
% constellations of the workspace convexness_structure_1 (K_1) region.
%
% IN THIS VERSION OF TRAINING SET CREATION VISITS ARBITRARY TRAJECTORIES ON
% THE POSSIBLE WORKSPACE OF BHAND. THIS WILL ALLOW IMPLEMENTATION OF
% ARBITRARY JUMPS IN THE WORKSPACE INTO THE TRAINING.
%
% IN THE TRAINING SET WHICH INCLUDES DISTINCT JUMPS of q VECTOR IS IMPLEMENTED 
% WITH RANDOM VISITING OF THE raster90 AND raster_flex VECTORS' ENTRIES. SET 
% INITIAL STATE WITH A RANDOM NUMBER AND WANDER ABOUT THE PREVIOUS STATE WITH
% ANOTHER RANDOM WIDTH:
    max_jump_angle = 5; % (5°, not radians or encoder steps) 
                        % This value directly influences the convergence of 
                        % the solution. In real time operation of the trained 
                        % net, the width of the jumps influences stability.

    trajectory_count  = 5;
    trajectory_length = 500;
    
    left_error_array      = zeros(trajectory_count, trajectory_length);
    right_error_array     = zeros(trajectory_count, trajectory_length);
    nonspread_error_array = zeros(trajectory_count, trajectory_length);

for trajectory_index = 1:trajectory_count

    % Initiate a vector index for the training set which will wander about in
    % the workspace:

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define the initial state q(0) with random steps
    %    
    % Here, the initialisation of the posture is repeated in each new loop,
    % different than a single initialisation in training set. There was the
    % discontinuity from a new initialisation was avoided to enable smooth
    % training. Here, the response of the network to discontinuities will
    % be observed. Therefore, new initialisations are allowed.
    
%     rwl_out_sim(1) = barrett.abduct_border * rand(1);
%     if( bhand.abduct.lower_R_bound > rwl_out_sim(1) )
%         rwl_out_sim(1) = bhand.abduct.lower_R_bound;
%     end
%     
%     for i = 2:4    
%         rwl_out_sim(i) = bhand.media.upper_R_bound * rand(1);
%         if( bhand.media.lower_R_bound > rwl_out_sim(i) )
%             rwl_out_sim(i) = bhand.media.lower_R_bound;
%         end
%     end

    % not random, constant initial posture:
    rwl_out_sim(1) = bhand.abduct.lower_R_bound + ...
                         ((barrett.abduct_border - bhand.abduct.lower_R_bound) / 2 );
    rwl_out_sim(2) = bhand.media.lower_R_bound;
    rwl_out_sim(3) = bhand.media.lower_R_bound;
    rwl_out_sim(4) = bhand.media.lower_R_bound;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now derive the "trajectory_index"th trajectory:
    for iteration = 1:trajectory_length
        
        % Trajectory will be built with differential stepping in joint
        % space:
        for i = 1:4
            rwl_out(i) = rwl_out_sim(i) + (max_jump_angle * pi * (rand(1) - 0.5))/180;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % angle2encoder routine converts the joint angle values of the vector
        % "rwl_out" to the motor encoder steps vector "motor_step". But at
        % this step the variable vector "rwl_out" will be overridden with help 
        % of rwl_out_sim. This is not a problem for the simulation, as the
        % bhand model is also overridden in this script.
        
            rwl_out_sim = rwl_out; % memorise the actual overridden rwl_out 
                                   % value for differential calculation of 
                                   % the next loop as this will be overwritten 
                                   % later in this loop by the real rwl_out 
                                   % variable.
        angle2encoder;
        
        bhand.ctrl.spread      = motor_step(1);
		bhand.ctrl.left.m      = motor_step(2);
		bhand.ctrl.right.m     = motor_step(3);
		bhand.ctrl.nonspread.m = motor_step(4);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Now map the encoder values back to the model:
        %
        bhand_control;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

        
        left_x_test      = bhand_q2cart.left_tip(x_axis);
        left_y_test      = bhand_q2cart.left_tip(y_axis);
        left_z_test      = bhand_q2cart.left_tip(z_axis);
        
        right_x_test     = bhand_q2cart.right_tip(x_axis);
        right_y_test     = bhand_q2cart.right_tip(y_axis);
        right_z_test     = bhand_q2cart.right_tip(z_axis);
       
        nonspread_x_test = bhand_q2cart.nonspread_tip(x_axis);
        nonspread_y_test = 0;
        nonspread_z_test = bhand_q2cart.nonspread_tip(z_axis);

        subplot(2,3,1),plot3(...
               bpalm_x, bpalm_y, bpalm_z, 'r',...
               bhand_q2cart.left(:,x_axis),      bhand_q2cart.left(:,y_axis),      bhand_q2cart.left(:,z_axis),...
               bhand_q2cart.right(:,x_axis),     bhand_q2cart.right(:,y_axis),     bhand_q2cart.right(:,z_axis),...
               bhand_q2cart.nonspread(:,x_axis), bhand_q2cart.nonspread(:,y_axis), bhand_q2cart.nonspread(:,z_axis))
        grid on
        axis([-150 150 -150 150 -150 150])
        xlabel('Quasi-Operator; Driver')
        ylabel('dimensions in mm')        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        desired_tip_coord_left      = bhand_q2cart.left(size(bhand_q2cart.nonspread,1),:);
        desired_tip_coord_right     = bhand_q2cart.right(size(bhand_q2cart.nonspread,1),:);
        desired_tip_coord_nonspread = bhand_q2cart.nonspread(size(bhand_q2cart.nonspread,1),:);
        
        % NOW DERIVE THE ABOVE ENDPOINTS CORRESPONDING to RWL JOINT VECTOR AND
        % SHOW THE POSTURE FOR THIS JOINT VECTOR. THAN THE TWO CAN BE
        % VISUALLY COMPARED.
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        simnet_input_vector = [ left_x_test;            left_y_test;         left_z_test;...
                               right_x_test;           right_y_test;        right_z_test;...
                               nonspread_x_test;   nonspread_x_test;    nonspread_z_test];
    
        % Simulate the backward kinematics network to derive guess joint angles vector:
        rwl_out = sim(net_rwl_points_elman,    simnet_input_vector);
        % [alfa_train; beta_left_train; beta_right_train; beta_nonspr_train] = rwl_out;   
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % angle2encoder routine converts the joint angle values of the vector
        % "rwl_out" to the motor encoder steps vector "motor_step".
        angle2encoder;
        
        bhand.ctrl.spread      = motor_step(1);
		bhand.ctrl.left.m      = motor_step(2);
		bhand.ctrl.right.m     = motor_step(3);
		bhand.ctrl.nonspread.m = motor_step(4);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Now map the encoder values back to the model:
        %
        bhand_control;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
        subplot(2,3,2),plot3(...
               bpalm_x, bpalm_y, bpalm_z, 'r',...
               bhand_q2cart.left(:,x_axis),      bhand_q2cart.left(:,y_axis),      bhand_q2cart.left(:,z_axis),...
               bhand_q2cart.right(:,x_axis),     bhand_q2cart.right(:,y_axis),     bhand_q2cart.right(:,z_axis),...
               bhand_q2cart.nonspread(:,x_axis), bhand_q2cart.nonspread(:,y_axis), bhand_q2cart.nonspread(:,z_axis))
        grid on
        axis([-150 150 -150 150 -150 150])
        xlabel('Estimated posture from net')
        ylabel('dimensions in mm')
        
        estimated_tip_coord_left      = bhand_q2cart.left(size(bhand_q2cart.nonspread,1),:);
        estimated_tip_coord_right     = bhand_q2cart.right(size(bhand_q2cart.nonspread,1),:);
        estimated_tip_coord_nonspread = bhand_q2cart.nonspread(size(bhand_q2cart.nonspread,1),:);
        
        left_error = estimated_tip_coord_left - desired_tip_coord_left;
        right_error = estimated_tip_coord_right - desired_tip_coord_right;
        nonspread_error = estimated_tip_coord_nonspread - desired_tip_coord_nonspread;
        
        subplot(2,3,4),bar(left_error,'b')
        xlabel('Left finger error(x,y,z) in mm')
        grid on
        axis([0 5 -50 50])
        
        subplot(2,3,5),bar(right_error,'g')
        xlabel('Right finger error(x,y,z) in mm')
        grid on
        axis([0 5 -50 50])
        
        subplot(2,3,6),bar(nonspread_error,'r')
        xlabel('Nonspread finger error(x,y,z) in mm')
        grid on
        axis([0 5 -50 50])
        
        F = getframe;
        
        left_error_array_x(trajectory_index, iteration) = left_error(x_axis);
        left_error_array_y(trajectory_index, iteration) = left_error(y_axis);
        left_error_array_z(trajectory_index, iteration) = left_error(z_axis);
        right_error_array_x(trajectory_index, iteration) = right_error(x_axis);
        right_error_array_y(trajectory_index, iteration) = right_error(y_axis);
        right_error_array_z(trajectory_index, iteration) = right_error(z_axis);
        nonspread_error_array_x(trajectory_index, iteration) = nonspread_error(x_axis);
        nonspread_error_array_y(trajectory_index, iteration) = nonspread_error(y_axis);
        nonspread_error_array_z(trajectory_index, iteration) = nonspread_error(z_axis);

        
    %iteration % show the step number        
    end 
            
end