%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Backward solution training
%
% After bhand finger distal link reaches its limit angle pi/2, the link
% does not move any more. After this border, the two link becomes a behaves
% like a single body and the cartesian trajectory on the flexion plane 
% becomes a circular arc. With the symmetric spread motion, all possible
% movements of the two spread fingers occur on a 1-eigth sphere in the 
% 1. quadrant of the cartesian space. 
%
% First derive the radius ro of the sphere of the trajectories, and run
% over all possible configurations for the spread fingers and save the
% corresponding postures. These will be used in training of a simple
% network for the backward kinematics solution. 
%
% The governing equations of posture are not seperable into the individual
% parameters and nonlinear. Therefore, this numeric solution is useful.

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
control_mode = 0
mode_b = 1;

barrett_hand;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First derive the training pairs:
train_pair_resolution = 22;        
raster90 = linspace(0,pi/2,train_pair_resolution);
raster_flex = linspace(pi/2 - atan2(56,70) , 7*pi/9, train_pair_resolution);
iteration = 0;

for abduct_index = 1:train_pair_resolution
    
    alfa = raster90(abduct_index);
    
    for left_motor_index = 1:train_pair_resolution
        
        beta_left = raster_flex(left_motor_index);
        
        for right_motor_index = 1:train_pair_resolution
            
            beta_right = raster_flex(right_motor_index);

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
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			
			    iteration = iteration + 1;
				barrett_fwd;
				barrett_points;
                
                left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
                left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
                left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
                
                right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
                right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
                right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
                

                alfa_train(iteration)       = alfa;
                beta_left_train(iteration)  = beta_left;
                beta_right_train(iteration) = beta_right;
       
                
                % Derived values are:
%                 c_point             = [(left_x_train(iteration) + right_x_train(iteration))/ 2 ...
%                                        (left_y_train(iteration) + right_y_train(iteration))/ 2 ...
%                                        (left_z_train(iteration) + right_z_train(iteration))/ 2];
% 			
%                 heigth              = sqrt(  ( c_point(x_axis) - nonspread_x_train(iteration) )^2 + ...
%                                              ( c_point(y_axis) - 0 )^2 +...
%                                              ( c_point(z_axis) - nonspread_x_train(iteration) )^2); % heigth of the mapping triangle
%                                          
%                 heigth_base_length  = sqrt(  ( left_x_train(iteration) - right_x_train(iteration) )^2 + ...
%                                              ( left_y_train(iteration) - right_y_train(iteration) )^2 + ...
%                                              ( left_z_train(iteration) - right_z_train(iteration) )^2 );
%                 
%                 center_of_gravity   = [ (left_x_train(iteration) + right_x_train(iteration) +  ...
%                                                             nonspread_x_train(iteration) )/ 3, ...
%                                         (left_y_train(iteration) + right_y_train(iteration))/ 3, ...
%                                         (left_z_train(iteration) + right_z_train(iteration) +  ...
%                                                             nonspread_z_train(iteration) )/ 3];
            end
        end
        iteration % show the step number
end

iteration = 1;
for nonspread_motor_index = 1:train_pair_resolution
    nonspread_right = raster_flex(nonspread_motor_index);

    barrett.left.abduct.rt_Q        = 0;
	barrett.left.media.rt_Q         = 0;
	barrett.left.distal.rt_Q        = 0;
	
    barrett.right.abduct.rt_Q       = 0;
    barrett.right.media.rt_Q        = 0;
	barrett.right.distal.rt_Q       = 0;
	
	  barrett.nonspread.media.rt_Q    = nonspread_right;
	  barrett.nonspread.distal.rt_Q   = pi/2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    iteration = iteration + 1;
	barrett_fwd;
	barrett_points;
                
   
   nonspread_x_train(iteration)= bhand_q2cart.nonspread_tip(x_axis);
   nonspread_z_train(iteration)= bhand_q2cart.nonspread_tip(z_axis);
end

save('bhand_rwl_train_pairs.mat', 'left_x_train',       'left_y_train',         'left_z_train', ...
                                  'right_x_train',      'right_y_train',        'right_z_train', ...
                                  'nonspread_x_train',  'nonspread_z_train',    'heigth',...
                                  'c_point',            'heigth_base_length',   'center_of_gravity', ...
                                  'alfa_train',         'beta_left_train',      'beta_right_train');

