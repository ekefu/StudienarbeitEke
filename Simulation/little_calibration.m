%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% little_calibration  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Boundary values for the little finger joints from medical literatur:                  
%
% The values are rough descriptive approximations to the movements of
% the fingers from medical literature, this can be of importance as a
% neighbourhood of convergence for the parameters of calibration, however,
% strict use of these values as they are makes the model less flexible. From
% adaptive modeling perspective, this could -for instance- be taken as an
% initialisation vector for the borders of value ranges.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	hhand.little.prox.rt_Q_range(max_val) = pi/2;                  
	hhand.little.media.rt_Q_range(max_val) = (5*pi)/9;
	hhand.little.distal.rt_Q_range(max_val) = pi/4;
	hhand.little.abduct.rt_Q_range(max_val) = pi/9; % A guess value; not found in medical literature. This should be calibrated. 
%    	hhand.little.abduct.rt_Q  = q_20_raster(angle_index);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The calibration should be carried out again for each new subject, indeed.
% Each hand can have variations from the "normal". 
%
% The following procedure for calibration of the max values from the cyber
% glove will be carried out with the special configuration of the hand,
% where all the digit fingers are flexed to the full, the finger tip
% pointing to the corresponding finger base, eahc finger constituting a
% roung triangle. This imaginary triangle can also be used to verify the
% finger link length measurements' compatibility with the measured joint
% angles.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% An alternative to the triangle approach may be the measurement of the
% angles in the fist configuration. In this configuration, the proximal
% flexion's max value can also be measured.
%
% hhand.little.prox.rt_Q_range(max_val) = cglove.little.prox.rt_Q;                  
% hhand.little.media.rt_Q_range(max_val) = cglove.little.media.rt_Q;
% hhand.little.distal.rt_Q_range(max_val) = cglove.little.distal.rt_Q;
% hhand.little.abduct.rt_Q_range(max_val) = cglove.little.abduct.rt_Q;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% According to the measured maximal joint angle values, the possible
% joint angle resolution can be distributed linearly as the following:
% 
% Cyber Glove; the realtime hhand joint angles measurement device, delivers 
% values with 8 bit resolution. Therefore number of max raster points
% is 128. A subset of this spektrum will be used, 
 
    hhand.little.prox.raster   = linspace(0,hhand.little.prox.rt_Q_range(max_val),angle_resolution);        % 90°
    hhand.little.media.raster  = linspace(0,hhand.little.media.rt_Q_range(max_val),angle_resolution);       % 100°
    hhand.little.distal.raster = linspace(0,hhand.little.distal.rt_Q_range(max_val),angle_resolution);      % 45°
  	hhand.little.abduct.raster = linspace(0,hhand.little.abduct.rt_Q_range(max_val),angle_resolution);      % 20°
    
%    hhandini alip öperim isiririm kopariririnrrrrrrrrrrrrrrrrrrrrrrrrrrr.