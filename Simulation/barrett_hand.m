%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A Barrett Hand Model
%
% The ground frame of reference is chosen to have its origin at the center
% of the palm. The x axis points towards the wrist, 
%
% left finger
% right finger
% nonspread
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proximal link lengths (taken from the producer's specifications; in mm)
    barrett.left.abduct.length = 50;
    barrett.right.abduct.length = 50;

% Medial link lengths
    barrett.left.media.length = 70;
    barrett.right.media.length = 70;
    barrett.nonspread.media.length = 70;

% Distal link lengths
    barrett.left.distal.length = 56;
    barrett.right.distal.length = 56;
    barrett.nonspread.distal.length = 56;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(mode_b == control_mode)
    angle_resolution_b = 20000;
else
    angle_resolution_b = 128;    % max possible resolution is 20000.
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Each link on the bhand model will be implemented according to a string model
% which has "length_resolution" times points including the beginning and
% end point.
	barrett.left.abduct.A_delta_x       = linspace(0, barrett.left.abduct.length,      length_resolution);
	barrett.left.media.A_delta_x        = linspace(0, barrett.left.media.length,       length_resolution);
	barrett.left.distal.A_delta_x       = linspace(0, barrett.left.distal.length,      length_resolution);
	
	barrett.right.abduct.A_delta_x      = linspace(0, barrett.right.abduct.length,     length_resolution);
	barrett.right.media.A_delta_x       = linspace(0, barrett.right.media.length,      length_resolution);
	barrett.right.distal.A_delta_x      = linspace(0, barrett.right.distal.length,     length_resolution);
	
	barrett.nonspread.media.A_delta_x   = linspace(0, barrett.nonspread.media.length,  length_resolution);
	barrett.nonspread.distal.A_delta_x  = linspace(0, barrett.nonspread.distal.length, length_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint space angle classes:
%
% The joint space of bhand is classified into the following groups all of
% which have individual uses:
%
% 1) possible angles range set P : this class of joint angles set includes all
% possible ranges of joint angles as given from the producer.
%
% 2) training angles range set T : this class of joint angles includes all the
% possible joint angle ranges which constitute a convex subspace of P. 
% ( T < P )
%
% 3) realtime angles range set R : this class of joint angles is restricted
% to a subset of T. This restriction is used to avoid bad estimates from
% the trained backward solution at the borders of the training set influence 
% realtime operation negatively.
% ( R < T ) => ( P < R < T ) 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1) possible angles range set P
%
% Full range of possible abduction (spread) angle spans a 180°s:
    barrett.abduct_border = pi;
	barrett.q_abduct      = linspace(0, barrett.abduct_border,   angle_resolution_b);
    
% Freespace angle-resolution for flexion joints: 
% flexion movement range/encoder resolution: 140°/20000
    barrett.q_free_flex1 = linspace(0, 7*pi/9, angle_resolution_b);
    
    for j = 1:angle_resolution_b
        
        ratio = pi/4 + 4.000*( barrett.q_free_flex1(j) )/3.000;
		if(ratio < pi/2)
            barrett.q_free_flex2(j) = ratio;
		end
		if(ratio >= pi/2)
            barrett.q_free_flex2(j) = pi/2;
		end
	end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2) training angles range set T
%
	bhand.abduct.lower_T_bound = pi/2;
	bhand.media.lower_T_bound  = pi/2 - atan2(56,70);
	bhand.media.upper_T_bound  = 135*pi/180 - atan2(56,70);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Derive the joint angle indexes (The following is used in "elman_rwl.m")
%
% First set the resolution of the workspace raster for the training set:
    train_pair_resolution = 23;   % training_pair_resolution is independent 
                                  % of motor resolution, because it is a 
                                  % function approximation parameter.
% Generate the training set abduction angle values spektrum:
    abduct_raster   = linspace( bhand.abduct.lower_T_bound,...
                                barrett.abduct_border,...
                                train_pair_resolution);
% Now generate the training set flexion angle values spektrum.    
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
% mimimum manipulability will be taken out.
    raster_flex = linspace( bhand.media.lower_T_bound,...
                            bhand.media.upper_T_bound,...
                            train_pair_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3) realtime angles range set R
% 
% These values will be a subset of the training range, to avoid bad
% estimates at the borders of the workspace points.
	bhand.abduct.lower_R_bound = (92 * pi)/180;      % 100° 
	bhand.media.lower_R_bound  = (92 * pi)/180 - atan2(56,70);
	bhand.media.upper_R_bound  = (133 * pi)/180 - atan2(56,70);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



bhand_rwl_net = 'rwl_points_elman';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If already trained and saved load the rwl network, if this is not the case,
% generate and train the backward solution network:
if( exist('load_net_rwl_points_elman.mat') )
% 	if(bhand_rwl_net == 'rwl_points_elman')
        load('load_net_rwl_points_elman.mat');
% 	end
% 	if(bhand_rwl_net == 'rwl_triangle_elman')
%         load('rwl_triangle_elman.mat');
% 	end
else
    train_rwl_points_elman;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%