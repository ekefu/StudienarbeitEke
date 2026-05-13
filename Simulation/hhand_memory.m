
if(with_memory)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First we need to initialise the memory elements which will keep the past
% cartesian positions of the hhand finger tips. The memory elements will
% later be used in the derivation of the mapping.
	hhand_q2cart.index_tip_past2 = 0;
	hhand_q2cart.middle_tip_past2 = 0;
	hhand_q2cart.ring_tip_past2 = 0;
	hhand_q2cart.little_tip_past2 = 0;
	hhand_q2cart.thumb_tip_past2 = 0;
    
  	hhand_q2cart.index_tip_past1 = 0;
	hhand_q2cart.middle_tip_past1 = 0;
	hhand_q2cart.ring_tip_past1 = 0;
	hhand_q2cart.little_tip_past1 = 0;
	hhand_q2cart.thumb_tip_past1 = 0;
    
end

    % Initialize the state vector:
    hhand_q2cart.index_tip = 0;
	hhand_q2cart.middle_tip = 0;
	hhand_q2cart.ring_tip = 0;
	hhand_q2cart.little_tip = 0;
	hhand_q2cart.thumb_tip = 0;
