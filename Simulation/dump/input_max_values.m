
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

max_search = 0;
linear_close = 1;

train_pair_resolution = 23;
    abduct_raster = linspace(0,bhand.abduct_T_bound,train_pair_resolution);
    raster_flex = linspace( bhand.media.lower_T_bound,...
                            bhand.media.upper_T_bound,...
                            train_pair_resolution);

if(max_search)
    iteration = 1;

    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = abduct_raster(1);
		  barrett.left.media.rt_Q         = raster_flex(1);
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = abduct_raster(1);
          barrett.right.media.rt_Q        = raster_flex(1);
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = 0;
		  barrett.nonspread.distal.rt_Q   = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
        
        iteration = iteration + 1;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = abduct_raster(train_pair_resolution);
		  barrett.left.media.rt_Q         = raster_flex(1);
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = abduct_raster(train_pair_resolution);
          barrett.right.media.rt_Q        = raster_flex(1);
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = 0;
		  barrett.nonspread.distal.rt_Q   = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
        
        
                iteration = iteration + 1;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = abduct_raster(train_pair_resolution);
		  barrett.left.media.rt_Q         = raster_flex(train_pair_resolution);
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = abduct_raster(train_pair_resolution);
          barrett.right.media.rt_Q        = raster_flex(1);
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = 0;
		  barrett.nonspread.distal.rt_Q   = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
        
                        iteration = iteration + 1;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = abduct_raster(train_pair_resolution);
		  barrett.left.media.rt_Q         = raster_flex(1);
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = abduct_raster(train_pair_resolution);
          barrett.right.media.rt_Q        = raster_flex(train_pair_resolution);
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = 0;
		  barrett.nonspread.distal.rt_Q   = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
        
                                iteration = iteration + 1;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = abduct_raster(1);
		  barrett.left.media.rt_Q         = raster_flex(1);
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = abduct_raster(1);
          barrett.right.media.rt_Q        = raster_flex(train_pair_resolution);
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = 0;
		  barrett.nonspread.distal.rt_Q   = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
        
                                     iteration = iteration + 1;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = abduct_raster(1);
		  barrett.left.media.rt_Q         = raster_flex(train_pair_resolution);
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = abduct_raster(1);
          barrett.right.media.rt_Q        = raster_flex(1);
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = 0;
		  barrett.nonspread.distal.rt_Q   = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);
end        

if(linear_close)
    
    for iteration=1:train_pair_resolution
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
          barrett.left.abduct.rt_Q        = abduct_raster(iteration);
		  barrett.left.media.rt_Q         = raster_flex(iteration);
		  barrett.left.distal.rt_Q        = pi/2;
		
          barrett.right.abduct.rt_Q       = abduct_raster(iteration);
          barrett.right.media.rt_Q        = raster_flex(iteration);
		  barrett.right.distal.rt_Q       = pi/2;
		
		  barrett.nonspread.media.rt_Q    = 0;
		  barrett.nonspread.distal.rt_Q   = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
        
        left_x_train(iteration)     = bhand_q2cart.left_tip(x_axis);
        left_y_train(iteration)     = bhand_q2cart.left_tip(y_axis);
        left_z_train(iteration)     = bhand_q2cart.left_tip(z_axis);
        
        right_x_train(iteration)    = bhand_q2cart.right_tip(x_axis);
        right_y_train(iteration)    = bhand_q2cart.right_tip(y_axis);
        right_z_train(iteration)    = bhand_q2cart.right_tip(z_axis);

        
        plot3(left_x_train, left_y_train, left_z_train,right_x_train, right_y_train, right_z_train);
        getframe;
    end
    
end