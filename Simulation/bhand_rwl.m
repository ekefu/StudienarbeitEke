%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Backward solution calculations for the bhand.
%                
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
% simnet_input_vector = [ left_x_test;            left_y_test;         left_z_test;...
%                         right_x_test;           right_y_test;        right_z_test;...
%                         nonspread_x_test;   nonspread_x_test;    nonspread_z_test];
% simnet_input_vector should be prepared prior to the call to this script
% with the mapping functions; not here. delete this part later.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
% Simulate the backward kinematics network to derive guess joint angles vector:
    rwl_out = sim(net_rwl_points_elman,    simnet_input_vector);
%
% rwl_out is like follows:    
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