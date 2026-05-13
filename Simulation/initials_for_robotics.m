%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initiation of simulation. Here will be the modes of the modules set.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
% The global named constants axes:
x_axis  = 1;
y_axis  = 2;
z_axis  = 3;

min_val = 1;
max_val = 2;

% Common link length grid resolution for bhand and hhand:
length_resolution = 5;

with_memory = 1;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % prepare hhand base properties
        human_hand;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Hhand control modes:
        synergic_close      = 1;
            simulation_angle_end_index   = 85;
            simulation_angle_start_index = 40;  % initial simulation index
            angle_index = simulation_angle_start_index;
    
        single_index_close  = 2;
        % random_move         = 3;

        mode_h              = synergic_close;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define bhand simulation modes (external control or linear simulation)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Bhand control modes:
        control_mode        = 1;     % Derive the finger points for mapped Q vector
        close_sim_mode      = 2;     % Simulate the close move
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hhand-Bhand mapping points derivation strategy:
% strategy1           = 1;        % use index,  ring   and thumb tips
% strategy2           = 2;        % use index,  little and thumb tips
% strategy3           = 3;        % use index-middle ring-little pairs' and thumb tips
% 
% triangle_strategy   = strategy3;
%
% COMMENTED BECAUSE, ALWAYS THE WEIGHTED STRATEGY3 WILL BE USED.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Strategy for jacobian derivation
    triangle_dimensions = 1;
    corner_coordinates  = 2;
    symmetry_frame_coordinates = 3;

    mode_j = corner_coordinates;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    log  = 0;

    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First, choose here the type of simulation you want to experience:

hhand_fwd_demo_mode  = 0;  % Shows a linear (linear joint angle) closing 
                           % demonstration of the human hand model.
                          
bhand_fwd_demo_mode  = 0;  % Shows a linear (linear joint angle) closing 
                           % demonstration of the barrett hand model.
                          
bhand_rwl_demo_mode  = 0;  % Representative backward kinematics demonstration; 
                           % does not show the generalization ability of the network.
                          
%bhand_rwl_train_mode = 0;  % Trains a bhand backward solution network 

teleopera_demo_mode  = 1;  % Shows the teleoperation system simulation. 

mapper_optimise_mode = 0;  % Shows the teleoperation system simulation 
                           % with mapper parameter optimisation algorithm together. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now, the necessary settings for the chosen simulation will be carried out
    if( hhand_fwd_demo_mode )        
        % Now set up hhand:
        mode_h              = synergic_close; 
      % mode_h              = single_index_close;
        simulation_angle_end_index   = 115;
        simulation_angle_start_index = 15;  % initial simulation index

        human_hand;
        for steps = 1:300
            
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
                  
                  F = getframe;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if( bhand_fwd_demo_mode )
        spread_offset = pi/2;
        bhand_fwd_demo;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if( bhand_rwl_demo_mode )
        bhand_rwl_demo;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if( teleopera_demo_mode )
        teleopera_mode;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if( mapper_optimise_mode )
        mapper_optimisation;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if( teleopera_demo_mode )
        teleopera_mode;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
