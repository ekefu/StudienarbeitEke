% optimise the map:

    
    
% 	mapper.barrett.rotate_z    = mapper.barrett.rotate_z_best + 0.0003 * 2 * (rand(1) - 0.5);
% 	mapper.barrett.rotate_y    = mapper.barrett.rotate_y_best + 0.0003 * 2 * (rand(1) - 0.5); 
% 	mapper.barrett.rotate_x    = mapper.barrett.rotate_x_best + 0.0003 * 2 * (rand(1) - 0.5);
% 	
% 	mapper.barrett.trans_x     = mapper.barrett.trans_x_best  + 0.1  * 2 * (rand(1) - 0.5);
% 	mapper.barrett.trans_y     = mapper.barrett.trans_y_best  + 0.1  * 2 * (rand(1) - 0.5);
	mapper.barrett.trans_z     = mapper.barrett.trans_z_best  + 0.1;
	
% 	mapper.hhand.index_weigth  = mapper.hhand.index_weigth_best  + 0.01 * 2 * (rand(1) - 0.5);
% 	mapper.hhand.middle_weigth = mapper.hhand.middle_weigth_best + 0.01 * 2 * (rand(1) - 0.5);
% 	mapper.hhand.ring_weigth   = mapper.hhand.ring_weigth_best   + 0.01 * 2 * (rand(1) - 0.5);
% 	mapper.hhand.little_weigth = mapper.hhand.little_weigth_best + 0.01 * 2 * (rand(1) - 0.5);
%     
    

fitness =( (corner_A(x_axis) - bhand_q2cart.left_tip(x_axis) )^2 + ...
               (corner_A(y_axis) - bhand_q2cart.left_tip(y_axis) )^2 + ...
               (corner_A(z_axis) - bhand_q2cart.left_tip(z_axis) )^2 + ...
               (corner_B(x_axis) - bhand_q2cart.right_tip(x_axis) )^2 + ...
               (corner_B(y_axis) - bhand_q2cart.right_tip(y_axis) )^2 + ...
               (corner_B(z_axis) - bhand_q2cart.right_tip(z_axis) )^2 + ...
               (corner_C(x_axis) - bhand_q2cart.nonspread_tip(x_axis) )^2 + ...
               (corner_C(y_axis) - bhand_q2cart.nonspread_tip(y_axis) )^2 + ...
               (corner_C(z_axis) - bhand_q2cart.nonspread_tip(z_axis) )^2  ) / 9;
	
	
	if( fitness < best_fitness )
        best_fitness = fitness;
        
%       mapper.barrett.rotate_z_best    = mapper.barrett.rotate_z;
% 		mapper.barrett.rotate_y_best    = mapper.barrett.rotate_y; 
% 		mapper.barrett.rotate_x_best    = mapper.barrett.rotate_x;
% 		
% 		mapper.barrett.trans_x_best     = mapper.barrett.trans_x;
% 		mapper.barrett.trans_y_best     = mapper.barrett.trans_y;
		mapper.barrett.trans_z_best     = mapper.barrett.trans_z;
% 		
% 		mapper.hhand.index_weigth_best  = mapper.hhand.index_weigth;
% 		mapper.hhand.middle_weigth_best = mapper.hhand.middle_weigth;
% 		mapper.hhand.ring_weigth_best   = mapper.hhand.ring_weigth;
% 		mapper.hhand.little_weigth_best = mapper.hhand.little_weigth;
    end
