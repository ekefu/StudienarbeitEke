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
% First derive the radius ro of the sphere of the trajectories, and run
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
% Derive the training pairs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% First set the resolution of the workspace raster for the training set:
    train_pair_resolution = 23;   % training_pair_resolution is independent 
                                  % of motor resolution, because it is a 
                                  % function approximation parameter.
%
% Generate the training set abduction angle values spektrum:
    abduct_raster = linspace(0,bhand.abduct_T_bound,train_pair_resolution);
%    
% Now generate the training set flexion angle values spektrum:    
%
% For the flexion angle restriction, a range of 90° - 135° is taken. This restricts 
% the workspace into one which is:
% 1) linear in circular coordinates (because of lower bound 90°, which takes the
% knee point out),
% 2) results in a meaningful synergic workspace of all fingers(because of
% the upper bound 135°, which takes the fingers crossed case out).
%
% The upper bound 135° for the media flexion angle is roughly above the
% point of minimum manipulability. A small neighbourhood of the point of
% mimimum manipulability will be taken out. BUNU HESAPLAYABILIRIM: xy
% izdüsümünün minimum oyun noktasi 
% see "barrett_hand.m".
    raster_flex = linspace( bhand.media.lower_T_bound,...
                            bhand.media.upper_T_bound,...
                            train_pair_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% Initiate a vector index for the training set which will wander about in
% the workspace:
    iteration = 0;
% Now start genarating training set I/O pairs, which will visit possible
% constellations of the workspace convexness_structure_1 (K_1) region.
%
% ANOTHER VERSION OF TRAINING SET CREATION SHOULD VISIT ARBITRARY POINTS ON
% THE POSSIBLE WORKSPACE OF BHAND. THIS WILL ALLOW IMPLEMENTATION OF
% ARBITRARY JUMPS IN THE WORKSPACE INTO THE TRAINING. IN THE FOLLOWING
% ALGORITHM, THE SET OF POINTS INSIDE THE POSSIBLE WORKSPACE ARE
% SEQUENTIALLY TRACED; THIS WON'T REFLECT THE ARBITRARY JUMPS. 
%
% HOWEVER; AS WE USE A BATCH-TRAINING ALGORITHM, THE SEQUENCE OF TRAINING
% PAIRS IS NOT IMPORTANT. BECAUSE THE ERROR BUILDING AND WEIGTH CHANGING HAPPENS 
% AFTER ALL THE TRAINING PAIRS ARE EVALUATED!
%
% IN SUCH A TRAINING SET WHICH WILL INCLUDE DISTINCT JUMPS q VECTOR OF THE
% JOINT SPACE, THE TRAINING SET SEQUENCE CAN BE IMPLEMENTED WITH RANDOM
% VISITING OF THE raster90 AND raster_flex VECTORS.
%
    randomised_set = 1;


    
    
for abduct_index = 1:train_pair_resolution
    
    alfa = abduct_raster(abduct_index);
    
    for left_motor_index = 1:train_pair_resolution
        
        beta_left = raster_flex(left_motor_index);
        
        for right_motor_index = 1:train_pair_resolution
            
            beta_right = raster_flex(right_motor_index);
            
%             for non_motor_index = 1:train_pair_resolution
%             
%                 beta_nonspread = raster_flex(non_motor_index);
        
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Multilinear tensor of the joint space of the barrett hand.           
                  barrett.left.abduct.rt_Q        = alfa;
				  barrett.left.media.rt_Q         = beta_left;
				  barrett.left.distal.rt_Q        = pi/2;
				
                  barrett.right.abduct.rt_Q       = alfa;
                  barrett.right.media.rt_Q        = beta_right;
				  barrett.right.distal.rt_Q       = pi/2;
				
				  barrett.nonspread.media.rt_Q    = 0;
				  barrett.nonspread.distal.rt_Q   = 0;
%                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			
			    iteration = iteration + 1;
				barrett_fwd;
				barrett_points;
                
                left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
                left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
                left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
                
                right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
                right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
                right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
%                
%                 nonspread_x_train(iteration)= bhand_q2cart.nonspread_tip(x_axis);
%                 nonspread_y_train(iteration)= 0;
%                 nonspread_z_train(iteration)= bhand_q2cart.nonspread_tip(z_axis);
% 
%
% Derive the desired network output values:
                alfa_train(iteration)       = alfa;
                beta_left_train(iteration)  = beta_left;
                beta_right_train(iteration) = beta_right;
%                
%                 % Derived values are:
%                 c_point(iteration,:)  = [(left_x_train(iteration) + right_x_train(iteration))/ 2 ...
%                                        (left_y_train(iteration) + right_y_train(iteration))/ 2 ...
%                                        (left_z_train(iteration) + right_z_train(iteration))/ 2];
% 			
%                 heigth(iteration)   = sqrt(  ( c_point(x_axis) - nonspread_x_train(iteration) )^2 + ...
%                                              ( c_point(y_axis) - 0 )^2 +...
%                                              ( c_point(z_axis) - nonspread_x_train(iteration) )^2); % heigth of the mapping triangle
%                                          
%                 heigth_base_length(iteration)  = sqrt(  ( left_x_train(iteration) - right_x_train(iteration) )^2 + ...
%                                              ( left_y_train(iteration) - right_y_train(iteration) )^2 + ...
%                                              ( left_z_train(iteration) - right_z_train(iteration) )^2 );
%                 
%                 center_of_gravity(iteration,:)   = [ (left_x_train(iteration) + right_x_train(iteration) +  ...
%                                                             nonspread_x_train(iteration) )/ 3, ...
%                                         (left_y_train(iteration) + right_y_train(iteration))/ 3, ...
%                                         (left_z_train(iteration) + right_z_train(iteration) +  ...
%                                                              nonspread_z_train(iteration) )/ 3];
%             end
        end
        iteration % show the step number
    end
    iteration
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

if(randomised_set)
    set_num = train_pair_resolution ^ 3;
    for ind_r = 1:set_num
        random_index = fix(set_num * rand(1));
        if(random_index > 0)
            r_left_x(ind_r)     = left_x_train(random_index);
            r_left_y(ind_r)     = left_y_train(random_index);
            r_left_z(ind_r)     = left_z_train(random_index);
            r_right_x(ind_r)    = right_x_train(random_index);
            r_right_y(ind_r)    = right_y_train(random_index);
            r_right_z(ind_r)    = right_z_train(random_index);
            r_alfa(ind_r)       = alfa_train(random_index);
            r_beta_left(ind_r)  = beta_left_train(random_index);
            r_beta_right(ind_r) = beta_right_train(random_index);
        end
    end
    
    left_x_train     = r_left_x;
    left_y_train     = r_left_y;
    left_z_train     = r_left_z;
    right_x_train    = r_right_x;
    right_y_train    = r_right_y;
    right_z_train    = r_right_z;
    alfa_train       = r_alfa;
    beta_left_train  = r_beta_left;
    beta_right_train = r_beta_right;

    clear('r_left_x', 'r_left_y', 'r_left_z', 'r_right_x', 'r_right_y', 'r_right_z', 'r_alfa', 'r_beta_left', 'r_beta_right');
end

save('randomised_upperset_train.mat', 'left_x_train', 'left_y_train', 'left_z_train', 'right_x_train', 'right_y_train', 'right_z_train', 'alfa_train', 'beta_left_train', 'beta_right_train');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% % Create the network for backward solution and train the network with the
% % dataset just created:
% %
% % Create the network to train:
% bh_rwl_rp_320_144_48_1500_rand_conv1 = newff([ min(left_x_train) max(left_x_train);...
%                                           min(left_y_train)  max(left_y_train);...
%                                           min(left_z_train)  max(left_z_train);...
%                                           min(right_x_train) max(right_x_train);...
%                                           min(right_y_train) max(right_y_train);...
%                                           min(right_z_train) max(right_z_train)],...
%                                           [320, 144, 48,  3],...
%                                           {'tansig', 'tansig', 'tansig', 'purelin'},...
%                                           'trainrp');
% % Set some training parameters:                 
%     bh_rwl_rp_320_144_48_1500_rand_conv1.trainParam.show   = 5;
%     %rwl_net.trainParam.lr     = 0.05;     % learning rate
%     %bhand.rwl_net.trainParam.lr_inc = 1.05;
%     %bhand.rwl_net.trainParam.mc     = 0.9;
%     bh_rwl_rp_320_144_48_1500_rand_conv1.trainParam.epochs = 1500;
% %
% % Train the network:
%     [bh_rwl_rp_320_144_48_1500_rand_conv1,tr] = train(bh_rwl_rp_320_144_48_1500_rand_conv1, [...
%                                          left_x_train;       left_y_train;       left_z_train;...
%                                         right_x_train;      right_y_train;      right_z_train],...
%                                         [  alfa_train;    beta_left_train;   beta_right_train]);       
% %
% % Save the trained network:
%     save('bh_rwl_rp_320_144_48_1500_rand_conv1.mat', 'bh_rwl_rp_320_144_48_1500_rand_conv1', 'tr');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% 
% 
% 
% clear('bh_rwl_rp_320_144_48_1500_rand_conv1.mat');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% Create the network for backward solution and train the network with the
% dataset just created:
%
% Create the network to train:
bh_rp_320_144_48_1500_27000_rand_conv1 = newelm([ min(left_x_train) max(left_x_train);...
                                          min(left_y_train)  max(left_y_train);...
                                          min(left_z_train)  max(left_z_train);...
                                          min(right_x_train) max(right_x_train);...
                                          min(right_y_train) max(right_y_train);...
                                          min(right_z_train) max(right_z_train)],...
                                          [48, 480, 64,  3],...
                                          {'tansig', 'tansig', 'tansig', 'purelin'},...
                                          'traingdx');
% Set some training parameters:                 
    bh_rp_320_144_48_1500_27000_rand_conv1.trainParam.show   = 5;
    bh_rwl_gdx_320_144_48_1500_007_107_rand_conv1.trainParam.lr     = 0.05;     % learning rate
    bh_rwl_gdx_320_144_48_1500_007_107_rand_conv1.trainParam.lr_inc = 1.05;
    bh_rwl_gdx_320_144_48_1500_007_107_rand_conv1.trainParam.mc     = 0.9;
    bh_rp_320_144_48_1500_27000_rand_conv1.trainParam.epochs = 500;
%
% Train the network:
    [bh_rp_320_144_48_1500_27000_rand_conv1,tr] = train(bh_rp_320_144_48_1500_27000_rand_conv1, [...
                                         left_x_train;       left_y_train;       left_z_train;...
                                        right_x_train;      right_y_train;      right_z_train],...
                                        [  alfa_train;    beta_left_train;   beta_right_train]);       
%
% Save the trained network:
    save('bh_rp_320_144_48_1500_27000_rand_conv1.mat', 'bh_rp_320_144_48_1500_27000_rand_conv1', 'tr');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    