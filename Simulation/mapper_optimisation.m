%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The mapper has the following 10 parameters for bhand-hhand palm alignment
% and mapping point selection. 
%
% Palm alignment homogeneous transformation matrix which implements a
% universal movement: raw, pitch, yaw orientation and x, y, z shift of the
% bhand origin based on a fitness function which evaluates the fitting of
% the two end point 3tuples.


% set a very large comparison value:
best_fitness = 999999999;

mode_j = corner_coordinates;

random_exploration = 1;
delta_exploration  = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First initialise the memory elements of the optimisation algorithm:
%
mapper.barrett.rotate_z_best    = -0.1;
mapper.barrett.rotate_y_best    = 0.1; 
mapper.barrett.rotate_x_best    = 0;

mapper.barrett.trans_x_best     = 10;
mapper.barrett.trans_y_best     = 0;
mapper.barrett.trans_z_best     = 10;
mapper.barrett.scaler_best      = 1;

% Mapping point selection weighting parameters; see "mapping_5_to_3.m".
mapper.hhand.index_weight_best  = 1;
mapper.hhand.middle_weight_best = 1;
mapper.hhand.ring_weight_best   = 1;
mapper.hhand.little_weight_best = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start the algorithm
% Use mutation-selection process to optimise the mapper parameters:
for search_count = 1:1500

	if(random_exploration)  
		mapper.barrett.rotate_z    = mapper.barrett.rotate_z_best + 0.01 * (rand(1) - 0.5);
		mapper.barrett.rotate_y    = mapper.barrett.rotate_y_best + 0.01 * (rand(1) - 0.5); 
		mapper.barrett.rotate_x    = mapper.barrett.rotate_x_best + 0.01 * (rand(1) - 0.5);
		
		mapper.barrett.trans_x     = mapper.barrett.trans_x_best  + 2  * (rand(1) - 0.5);
		mapper.barrett.trans_y     = mapper.barrett.trans_y_best  + 2  * (rand(1) - 0.5);
		mapper.barrett.trans_z     = mapper.barrett.trans_z_best  + 2  * (rand(1) - 0.5);
        mapper.barrett.scaler      = mapper.barrett.scaler_best  + 0.1 * (rand(1) - 0.5);
		
		mapper.hhand.index_weight  = mapper.hhand.index_weight_best  + 0.01 * (rand(1) - 0.5);
		mapper.hhand.middle_weight = mapper.hhand.middle_weight_best + 0.01 * (rand(1) - 0.5);
		mapper.hhand.ring_weight   = mapper.hhand.ring_weight_best   + 0.01 * (rand(1) - 0.5);
		mapper.hhand.little_weight = mapper.hhand.little_weight_best + 0.01 * (rand(1) - 0.5);
	end
    
    % Now execute the teleoperation simulation to see the results from the
    % actual mapper parameters:
    teleopera_mode;
	% fitness is derived in the "teleopera_mode" script.
    
    % now the calculated cumulative loop-fitness as a compare measure:
    fitness = loop_fitness;
    
	
	if( fitness < best_fitness )
        best_fitness = fitness;
        
        mapper.barrett.rotate_z_best    = mapper.barrett.rotate_z;
		mapper.barrett.rotate_y_best    = mapper.barrett.rotate_y; 
		mapper.barrett.rotate_x_best    = mapper.barrett.rotate_x;
		
		mapper.barrett.trans_x_best     = mapper.barrett.trans_x;
		mapper.barrett.trans_y_best     = mapper.barrett.trans_y;
		mapper.barrett.trans_z_best     = mapper.barrett.trans_z;
        mapper.barrett.scaler_best      = mapper.barrett.scaler;
		
		mapper.hhand.index_weight_best  = mapper.hhand.index_weight;
		mapper.hhand.middle_weight_best = mapper.hhand.middle_weight;
		mapper.hhand.ring_weight_best   = mapper.hhand.ring_weight;
		mapper.hhand.little_weight_best = mapper.hhand.little_weight;
	end
end


save('mapper_optimised.mat', '')