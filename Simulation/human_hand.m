%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A Human Hand Model (LEFT HAND)
%
% The ground frame of reference is chosen to have its origin at the center
% of the palm. The x axis points towards the wrist, 
%
% f1: index finger
% f2: middle finger
% f3: ring finger
% f4: little finger
% f5: thumb
%
% Human hand link lenghts are given as a rough example for modeling 
% a simulation environment. Better measurements must be done prior to 
% hardware execution.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Proximal link lengths (measure from the inner side of the hand; in mm)
hhand.index.prox.length = 28;
hhand.middle.prox.length = 30;
hhand.ring.prox.length = 30;
hhand.little.prox.length = 25;
hhand.thumb.prox.length = 45;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Medial link lengths
hhand.index.media.length = 20;
hhand.middle.media.length = 25;
hhand.ring.media.length = 25;
hhand.little.media.length = 15;
hhand.thumb.media.length = 20;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Distal link lengths
hhand.index.distal.length = 15;
hhand.middle.distal.length = 20;
hhand.ring.distal.length = 20;
hhand.little.distal.length = 15;
hhand.thumb.distal.length = 15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The base joint translational offsets will be taken to be constant and
% coplanar on the palm. These offsets will be assumed to be invariant with
% respect to finger movements, and hence the hand control.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Left Hand base points of fingers, w.r.t. palm center
% seen from inner side of the palm.
hhand.thumb.base.point  = [ 30 -15 0];
hhand.index.base.point  = [-30 -30 0];
hhand.middle.base.point = [-35 -10 0];
hhand.ring.base.point   = [-35  10 0];
hhand.little.base.point = [-30  30 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Symmetrical point of the thumb base at the opposite side.
palm_corner = [30 30 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Visit all the finger bases including the fpcorner, beginning and ending
% % in thumb base. (thumb, index, middle, .. .. palm_corner, thumb)
palm_x = [ -30  30  35  35  30  -30 -30]; 
palm_y = [  15  30  10 -10 -30  -30  15];
palm_z = [  0   0   0   0   0    0   0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Angle resolution for the simulated human hand joint angles are taken to be equal to 128. 
% This value is 1-byte range for the individual joint angles as being delivered from the
% cyber glove instrument.
%
% The length resolution is not very important in free space simulation,
% however, it can be necessary to increase the density of points in an
% interaction simulation, in which virtual object manipulation takes place.
%
angle_resolution = 128;
% length_resolution = 5; 
% length_resolution is common to bhand and hhand for now, and hence is
% being set in file initials_for_robotics.m like a global parameter.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Generate the basis frame of references for the fingers:
hhand.index.base.A  = trans(hhand.index.base.point(1),  hhand.index.base.point(2),  hhand.index.base.point(3));
hhand.middle.base.A = trans(hhand.middle.base.point(1), hhand.middle.base.point(2), hhand.middle.base.point(3));
hhand.ring.base.A   = trans(hhand.ring.base.point(1),   hhand.ring.base.point(2),   hhand.ring.base.point(3));
hhand.little.base.A = trans(hhand.little.base.point(1), hhand.little.base.point(2), hhand.little.base.point(3));
hhand.thumb.base.A  = trans(hhand.thumb.base.point(1),  hhand.thumb.base.point(2),  hhand.thumb.base.point(3));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate arrays of simulation points for the bodies (links) comprising the
% multi-body hand structure. Each link will be represented with 5 equally
% distributed points along the length of the link.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% index finger link points:
hhand.index.prox.A_delta_x    = linspace(0, hhand.index.prox.length,    length_resolution);
hhand.index.media.A_delta_x   = linspace(0, hhand.index.media.length,   length_resolution);
hhand.index.distal.A_delta_x  = linspace(0, hhand.index.distal.length,  length_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% middle finger link points:
hhand.middle.prox.A_delta_x   = linspace(0, hhand.middle.prox.length,   length_resolution);
hhand.middle.media.A_delta_x  = linspace(0, hhand.middle.media.length,  length_resolution);
hhand.middle.distal.A_delta_x = linspace(0, hhand.middle.distal.length, length_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ring finger link points:
hhand.ring.prox.A_delta_x     = linspace(0, hhand.ring.prox.length,     length_resolution);
hhand.ring.media.A_delta_x    = linspace(0, hhand.ring.media.length,    length_resolution);
hhand.ring.distal.A_delta_x   = linspace(0, hhand.ring.distal.length,   length_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% little finger link points:
hhand.little.prox.A_delta_x   = linspace(0, hhand.little.prox.length,   length_resolution);
hhand.little.media.A_delta_x  = linspace(0, hhand.little.media.length,  length_resolution);
hhand.little.distal.A_delta_x = linspace(0, hhand.little.distal.length, length_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% thumb finger link points:
hhand.thumb.prox.A_delta_x    = linspace(0, hhand.thumb.prox.length,    length_resolution); 
hhand.thumb.media.A_delta_x   = linspace(0, hhand.thumb.media.length,   length_resolution);
hhand.thumb.distal.A_delta_x  = linspace(0, hhand.thumb.distal.length,  length_resolution);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now call the joint angle ranges set up procedures, which will calibrate
% the read values from cyber glove to the variablities in the operator
% hand.
%
% In the simulation, the maximum values are used for the proximal, media
% and distal finger joints found from the medical course book:
% Lippert, "Anatomie". For the joints to which no maximal angle value
% found, a rough estimate value is used for now, these values should be
% replaced with experimental ones for each operator.
	index_calibration;
	middle_calibration;
	ring_calibration;
	little_calibration;
	thumb_calibration;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialise the memory elements, if with memory
hhand_memory;