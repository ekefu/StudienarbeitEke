%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Backward solution training 
%
% (called from <- barrett_hand.m <- initials_for_robotics.m)
% 
% After bhand finger distal link reaches its limit angle pi/2, the link
% does not move any more. After this border, the two link behaves
% like a single body and the cartesian trajectory on the flexion plane 
% becomes a circular arc. With the symmetric spread motion, all possible
% movements of the two spread fingers occur on an inner circumference of 
% a torus slice. Analytical equations expressing this workspace can be
% expressed in spherical coordinates, but however, the variables from such
% equation systems are not seperable into simple ODEs. Therefore, a
% numerical treatment will be carried out to find the backward
% solution.
% First derive the radius of the sphere of the trajectories, and run
% over all possible configurations for the spread fingers and save the
% corresponding postures. These will be used in training of a simple
% network for the backward kinematics solution. 
% 
% The governing equations of posture are not seperable into the individual
% parameters and nonlinear. Therefore, this numeric solution is useful.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create the network for backward solution
%
net_rwl_points_elman = newelm([  -20    70;...
                                 -15    75;...
                                  60    90;...
                                 -20    70;...
                                 -75    15;...
                                  60    90;...
                                 -30    30;...
                                 -30    30;...
                                  60    90],...
                                 [240, 480, 128,  4],...
                                 {'tansig', 'tansig', 'tansig', 'purelin'},...
                                 'traingdx');
                 
% Set some training parameters:                 
%     net_rwl_points_elman.trainParam.show   = 5;
%     net_rwl_points_elman.trainParam.lr     = 0.05;     % learning rate
%     net_rwl_points_elman.trainParam.lr_inc = 1.05;
%     net_rwl_points_elman.trainParam.mc     = 0.9;
%     net_rwl_points_elman.trainParam.epochs = 100;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    max_jump_width = 7; % this value directly influences the convergence of the solution.
                        % in real time operation of the trained net, the
                        % width of the jumps influences stability.

    trajectory_count  = 50;
    trajectory_length = 50;

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define the initial state q(0) with random steps
    %    
    % Here, the initialisation of the posture is not repeated in each new loop,
    % The discontinuity from a new initialisation is avoided to enable smooth
    % training.
    
        alfa_index        = fix(train_pair_resolution / 2);
        beta_left_index   = 1;
        beta_right_index  = 1;
        beta_nonspr_index = 1;

%         alfa_index        = fix(train_pair_resolution * (rand(1) - 0.5));
%         beta_left_index   = fix(train_pair_resolution * (rand(1) - 0.5));
%         beta_right_index  = fix(train_pair_resolution * (rand(1) - 0.5));
%         beta_nonspr_index = fix(train_pair_resolution * (rand(1) - 0.5));

%     % not random, use constant initial posture: This will be the very same 
%     % initial posture for realtime operation and simulation:
%         rwl_out_sim(1) = fix((barrett.abduct_border - bhand.abduct.lower_R_bound) / 2 );
%         rwl_out_sim(2) = bhand.media.lower_R_bound;
%         rwl_out_sim(3) = bhand.media.lower_R_bound;
%         rwl_out_sim(4) = bhand.media.lower_R_bound;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
for trajectory_index = 1:trajectory_count

    % Generate a vector index for the training set which will wander about in
    % the workspace:

    % Now derive the "trajectory_index"th trajectory:
    for iteration = 1:trajectory_length
        
        % Trajectory will be built with differential stepping in joint
        % space:
        alfa_index = alfa_index + fix(max_jump_width * (rand(1) - 0.5));
        % check index boundaries joint angle vectors:
        if (alfa_index > train_pair_resolution)
            alfa_index = train_pair_resolution;
        end
        if (alfa_index < 1)
            alfa_index = 1;
        end
        
        beta_left_index = beta_left_index + fix(max_jump_width * (rand(1) - 0.5));
        if (beta_left_index > train_pair_resolution)
            beta_left_index = train_pair_resolution;
        end
        if (beta_left_index < 1)
            beta_left_index = 1;
        end
        
        beta_right_index = beta_right_index + fix(max_jump_width * (rand(1) - 0.5));
        if (beta_right_index > train_pair_resolution)
            beta_right_index = train_pair_resolution;
        end
        if (beta_right_index < 1)
            beta_right_index = 1;
        end
        
        beta_nonspr_index = beta_nonspr_index + fix(max_jump_width * (rand(1) - 0.5));
        if (beta_nonspr_index > train_pair_resolution)
            beta_nonspr_index = train_pair_resolution;
        end
        if (beta_nonspr_index < 1)
            beta_nonspr_index = 1;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Now set the joint space angles according to the generated indexes
        %
        alfa         = abduct_raster(alfa_index);
        beta_left    = raster_flex(beta_left_index);
        beta_right   = raster_flex(beta_right_index);    
	    beta_nonspr  = raster_flex(beta_nonspr_index);    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = alfa;
		  barrett.left.media.rt_Q         = beta_left;
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = alfa;
          barrett.right.media.rt_Q        = beta_right;
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = beta_nonspr;
		  barrett.nonspread.distal.rt_Q   = pi/2;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)      = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)      = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)      = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)     = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)     = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)     = bhand_q2cart.right_tip(z_axis);
       
        nonspread_x_train(iteration) = bhand_q2cart.nonspread_tip(x_axis);
        nonspread_y_train(iteration) = 0;
        nonspread_z_train(iteration) = bhand_q2cart.nonspread_tip(z_axis);

        % Record also the desired network output values:
        alfa_train(iteration)        = alfa;
        beta_left_train(iteration)   = beta_left;
        beta_right_train(iteration)  = beta_right;
        beta_nonspr_train(iteration) = beta_nonspr;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % if the mapping vector includes the dimensions of the finger tips
        % triangle, they have to be calculated here:
        if(mode_j == triangle_dimensions)
            % Derived values are:
            c_point(iteration,:)  = [(left_x_train(iteration) + right_x_train(iteration))/ 2 ...
                                     (left_y_train(iteration) + right_y_train(iteration))/ 2 ...
                                     (left_z_train(iteration) + right_z_train(iteration))/ 2];
	    
            % heigth of the mapping triangle
            heigth(iteration)   = sqrt(  ( c_point(x_axis) - nonspread_x_train(iteration) )^2 + ...
                                         ( c_point(y_axis) - 0 )^2 +...
                                         ( c_point(z_axis) - nonspread_x_train(iteration) )^2);
                                 
            % base length of the mapping triangle                         
            heigth_base_length(iteration)  = sqrt(  ( left_x_train(iteration) - right_x_train(iteration) )^2 + ...
                                                    ( left_y_train(iteration) - right_y_train(iteration) )^2 + ...
                                                    ( left_z_train(iteration) - right_z_train(iteration) )^2 );

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % if the mapping vector includes endpoint coordinates at symmetry frame, 
        % they have to be calculated here:
        if(mode_j == symmetry_frame_coordinates)
            
            
            
            corner_A_at_sym_frame_4     = A_sym_frame * [ corner_A(x_axis) corner_A(y_axis) corner_A(z_axis)   1]';
            corner_A_at_sym_frame(iteration)       =    [ corner_A_at_sym_frame_4(x_axis)...
                                                          corner_A_at_sym_frame_4(y_axis)...
                                                          corner_A_at_sym_frame_4(z_axis)];
            
            corner_B_at_sym_frame_4     = A_sym_frame * [ corner_B(x_axis) corner_B(y_axis) corner_B(z_axis)   1]';
            corner_B_at_sym_frame       = [ corner_B_at_sym_frame_4(x_axis)...
                                            corner_B_at_sym_frame_4(y_axis)...
                                            corner_B_at_sym_frame_4(z_axis) ];
                                    
            corner_C_at_sym_frame       = projected_thumb_at_sym_frame;                                
	
            world_origin_at_sym_frame_4 = A_sym_frame * [0 0 0 1]';
            world_origin_at_sym_frame   = [ world_origin_at_sym_frame_4(x_axis)...
                                            world_origin_at_sym_frame_4(y_axis)...
                                            world_origin_at_sym_frame_4(z_axis)];
                                    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if(mode_j == symmetry_frame_coordinates)
                net_input_vector = [ corner_A_at_sym_frame(x_axis);...
                                    corner_A_at_sym_frame(y_axis);...
                                    corner_A_at_sym_frame(z_axis);...
                                    corner_B_at_sym_frame(x_axis);...
                                    corner_B_at_sym_frame(y_axis);...
                                    corner_B_at_sym_frame(z_axis);...
                                    corner_C_at_sym_frame(x_axis);...
                                    corner_C_at_sym_frame(y_axis);...
                                    corner_C_at_sym_frame(z_axis) ];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	
            
    %iteration % show the step number        
    end

    net_input_vector = [ left_x_train;            left_y_train;         left_z_train;...
                        right_x_train;           right_y_train;        right_z_train;...
                        nonspread_x_train;   nonspread_y_train;    nonspread_z_train];
    
    % Train the network:
    [net_rwl_points_elman ,tr] = train(net_rwl_points_elman,    net_input_vector,...
                                    [      alfa_train;     beta_left_train;...
                                     beta_right_train;   beta_nonspr_train]);       
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save the trained network:
    save('load_net_rwl_points_elman.mat', 'net_rwl_points_elman', 'tr');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%