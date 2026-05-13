%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "angle2encoder.m" 
% This skript converts the angle values out of the backward kinematics
% network for the bhand into proper motor encoder values. It takes the
% network output vector of bhand joint angles and gives the corresponding
% bhand motor encoders vector.

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    % THE OUTPUTS FROM THE BACKWARD SOLUTION NETWORK ARE CONDITIONED 
    % in 2 steps:
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 1) TRIM THE VALUES BEYOND THE POSSIBLE ANGLE RANGES
        %
        % Take the boundary values, if the derived angle values are outside 
        % the realtime boundaries:
        
        % abduction angle       
        if(rwl_out(1) < bhand.abduct.lower_R_bound)
            rwl_out(1) = bhand.abduct.lower_R_bound;
        end
        if(rwl_out(1) > barrett.abduct_border)
            rwl_out(1) = barrett.abduct_border;
        end
        
        for i=2:4   
        % left flex, rigth flex, nonspread flex
            if(rwl_out(i)< bhand.media.lower_R_bound)
                rwl_out(i) = bhand.media.lower_R_bound;
            end
            if(rwl_out(i) > bhand.media.upper_R_bound)
                rwl_out(i) = bhand.media.upper_R_bound;
            end
        end

    	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % 2) CONVERT THE ANGLE VALUES INTO MOTOR ENCODER VALUES        
        %
        % The matching encoder steps are approximated with the following
        % linear equations. As the motor encoder values have to be
        % integers, the resulting values are taken to the next integer.
        %
        % motor_step_abduct = (abduct_joint_angle * 20000) / pi;
		% =>
          motor_step(1) = fix(rwl_out(1) * 6366.1977);
          
		% motor_step_flex = ((flex_joint_angle * 20000) * 9) / (7 * pi);
		% =>
          motor_step(2) = fix(rwl_out(2) * 8185.1114);
          motor_step(3) = fix(rwl_out(3) * 8185.1114);
          motor_step(4) = fix(rwl_out(4) * 8185.1114);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%