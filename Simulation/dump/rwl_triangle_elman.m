%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Backward solution training (SELF-CONTAINED)
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

clear;

% The global named constants axes:
x_axis = 1;
y_axis = 2;
z_axis = 3;

% Named constants for transformation functions:
vector = 0;
matrix = 1;

min_val = 1;
max_val = 2;

% Common link length grid resolution for bhand and hhand:
length_resolution = 5;

barrett_hand;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% Create the network for backward solution
%
rwl_triangle_elman = newelm([-20    70;...
                             -15    75;...
                              60    90;...
                             -20    70;...
                             -75    15;...
                              60    90],...
                             [480, 128,  4],...
                             {'tansig', 'tansig', 'purelin'},...
                             'traingdx');
                 
% Set some training parameters:                 
    rwl_triangle_elman.trainParam.show   = 5;
    rwl_triangle_elman.trainParam.lr     = 0.05;     % learning rate
    rwl_triangle_elman.trainParam.lr_inc = 1.05;
    rwl_triangle_elman.trainParam.mc     = 0.9;
    rwl_triangle_elman.trainParam.epochs = 50;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    
    
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
    max_jump_width = 5; % this value directly influences the convergence of the solution.
                        % in real time operation of the trained net, the
                        % width of the jumps influences stability.

    trajectory_count  = 50;
    trajectory_length = 50;

for trajectory_index = 1:trajectory_count

    % Initiate a vector index for the training set which will wander about in
    % the workspace:

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define the initial state q(0) with random steps
        alfa_index = fix(train_pair_resolution * rand(1));
        beta_left_index = fix(train_pair_resolution * rand(1));
        beta_right_index = fix(train_pair_resolution * rand(1));
        beta_nonspr_index = fix(train_pair_resolution * rand(1));        

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now derive the "trajectory_index"th trajectory:
    for iteration = 1:trajectory_length
        
        % Trajectory will be built with differential stepping in joint
        % space:
        alfa_index = alfa_index + fix(max_jump_width * rand(1));
        % check index boundaries joint angle vectors:
        if (alfa_index > train_pair_resolution)
            alfa_index = train_pair_resolution;
        end
        if (alfa_index < 1)
            alfa_index = 1;
        end
        
        beta_left_index = beta_left_index + fix(max_jump_width * rand(1));
        if (beta_left_index > train_pair_resolution)
            beta_left_index = train_pair_resolution;
        end
        if (beta_left_index < 1)
            beta_left_index = 1;
        end
        
        beta_right_index = beta_right_index + fix(max_jump_width * rand(1));
        if (beta_right_index > train_pair_resolution)
            beta_right_index = train_pair_resolution;
        end
        if (beta_right_index < 1)
            beta_right_index = 1;
        end
        
        beta_nonspr_index = beta_nonspr_index + fix(max_jump_width * rand(1));
        if (beta_nonspr_index > train_pair_resolution)
            beta_nonspr_index = train_pair_resolution;
        end
        if (beta_nonspr_index < 1)
            beta_nonspr_index = 1;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
	               
	    nonspread_x_train(iteration)= bhand_q2cart.nonspread_tip(x_axis);
	    nonspread_y_train(iteration)= 0;
	    nonspread_z_train(iteration)= bhand_q2cart.nonspread_tip(z_axis);
      
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

        % Record also the desired network output values:
        alfa_train(iteration)       = alfa;
        beta_left_train(iteration)  = beta_left;
        beta_right_train(iteration) = beta_right;
        
        
    iteration % show the step number        
    end

    net_input_vector = [ left_z_train;          right_z_train;      nonspread_z_train;...
                         heigth_base_length;    heigth;             c_point],...
                     
    % Train the network:
    [rwl_triangle_elman ,tr] = train(rwl_triangle_elman, net_input_vector,...
                                    [alfa_train;         beta_left_train;...
                                     beta_right_train;   beta_nonspr_train]);           
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% Save the trained network:
    save('rwl_triangle_elman.mat', 'rwl_elman', 'tr');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    