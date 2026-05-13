%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% control.m reduces the joint space of the barrett hand into the control 
% space according to the free space behaviour of the TorqueSwitch
% mechanism. For the details of the mechanism, see the D0AA8A.pdf
% document under:
%
% http://www.barrett.com/robot/products/hand/D0AA8A.pdf
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TO CHANGE MODES, SEE: INITIALS_FOR_ROBOTICS.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


switch mode_b 
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % close_sim_mode maps simulates linear close movement of bhand.
    case close_sim_mode
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
        barrett.left.abduct.rt_Q        = barrett.q_abduct(angle_index); % angle_resolution_b/2 + barrett.abduct_border
		barrett.left.media.rt_Q         = barrett.q_free_flex1(angle_index);
		barrett.left.distal.rt_Q        = barrett.q_free_flex2(angle_index);
		
        barrett.right.abduct.rt_Q       = barrett.q_abduct(angle_index);
        barrett.right.media.rt_Q        = barrett.q_free_flex1(angle_index);
		barrett.right.distal.rt_Q       = barrett.q_free_flex2(angle_index);
		
		barrett.nonspread.media.rt_Q    = barrett.q_free_flex1(angle_index);
		barrett.nonspread.distal.rt_Q   = barrett.q_free_flex2(angle_index);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    
    
    
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control_mode maps the current hhand onto the bhand.
    case control_mode
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Multilinear tensor of the joint space of the barrett hand.           
        barrett.left.abduct.rt_Q        = barrett.q_abduct(bhand.ctrl.spread);
		barrett.left.media.rt_Q         = barrett.q_free_flex1(bhand.ctrl.left.m);
		barrett.left.distal.rt_Q        = barrett.q_free_flex2(bhand.ctrl.left.m);
		
        barrett.right.abduct.rt_Q       = barrett.q_abduct(bhand.ctrl.spread);
        barrett.right.media.rt_Q        = barrett.q_free_flex1(bhand.ctrl.right.m);
		barrett.right.distal.rt_Q       = barrett.q_free_flex2(bhand.ctrl.right.m);
		
		barrett.nonspread.media.rt_Q    = barrett.q_free_flex1(bhand.ctrl.nonspread.m);
		barrett.nonspread.distal.rt_Q   = barrett.q_free_flex2(bhand.ctrl.nonspread.m);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
		barrett_fwd;
		barrett_points;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
end