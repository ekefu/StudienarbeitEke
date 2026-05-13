%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start the control loop this simulates a single step of interaction. to
% enable loop operation with the specified mode, call this script from
% inside a loop in another script.

% TO CHANGE MODES, SEE: INITIALS_FOR_ROBOTICS.M
switch mode_h 
    
    case synergic_close
        
        if( angle_index > simulation_angle_end_index )
            angle_index = simulation_angle_start_index;
        else
            angle_index = angle_index + 1;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        % INDEX FINGER
            hhand.index.prox.rt_Q   = hhand.index.prox.raster(angle_index);                  
		    hhand.index.media.rt_Q  = hhand.index.media.raster(angle_index);
		    hhand.index.distal.rt_Q = hhand.index.distal.raster(angle_index);
		    hhand.index.abduct.rt_Q = hhand.index.abduct.raster(angle_index);
        
        % Now calculate the world points in cartesian space corresponding
        % to this posture:
        index_fwd;
        index_points;
        
            if(log)
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % the following may be used in model-verification:
                w.hhand.index.previous = hhand_q2cart.index(length_index + length_resolution * 2, :);
                index_z_of_m(angle_index) = hhand_q2cart.index(15, z_axis);
                index_x_of_m(angle_index) = hhand_q2cart.index(15, x_axis);
                index_y_of_m(angle_index) = hhand_q2cart.index(15, y_axis);
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
            
	
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        %MIDDLE FINGER
            hhand.middle.prox.rt_Q    = hhand.middle.prox.raster(angle_index);                  
			hhand.middle.media.rt_Q   = hhand.middle.media.raster(angle_index);
			hhand.middle.distal.rt_Q  = hhand.middle.distal.raster(angle_index);
			hhand.middle.abduct.rt_Q  = hhand.middle.abduct.raster(angle_index);
                
        % Now calculate the world points in cartesian space correspoing
        % to this posture:
        middle_fwd;
        middle_points;
        
            if(log)        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % the following may be used in model-verification:
                w.hhand.middle.previous = hhand_q2cart.middle(length_index + length_resolution * 2, :);
                middle_z_of_m(angle_index) =   hhand_q2cart.middle(15, z_axis);
                middle_x_of_m(angle_index) =   hhand_q2cart.middle(15, x_axis);
                middle_y_of_m(angle_index) =   hhand_q2cart.middle(15, y_axis);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        %RING FINGER
            hhand.ring.prox.rt_Q    = hhand.ring.prox.raster(angle_index);                  
			hhand.ring.media.rt_Q   = hhand.ring.media.raster(angle_index);
			hhand.ring.distal.rt_Q  = hhand.ring.distal.raster(angle_index);
			hhand.ring.abduct.rt_Q  = hhand.ring.abduct.raster(angle_index);
                
        % Now calculate the world points in cartesian space correspoing
        % to this posture:
        ring_fwd;
        ring_points;
        
            if(log)        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % the following may be used in model-verification:
                w.hhand.ring.previous = hhand_q2cart.ring(length_index + length_resolution * 2, :);
                ring_z_of_m(angle_index) =   hhand_q2cart.ring(15, z_axis);
                ring_x_of_m(angle_index) =   hhand_q2cart.ring(15, x_axis);
                ring_y_of_m(angle_index) =   hhand_q2cart.ring(15, y_axis);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        %LITTLE FINGER
            hhand.little.prox.rt_Q    = hhand.little.prox.raster(angle_index);                  
			hhand.little.media.rt_Q   = hhand.little.media.raster(angle_index);
			hhand.little.distal.rt_Q  = hhand.little.distal.raster(angle_index);
			hhand.little.abduct.rt_Q  = hhand.little.abduct.raster(angle_index);
                
        % Now calculate the world points in cartesian space correspoing
        % to this posture:
        little_fwd;
        little_points;
            
            if(log)        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % the following may be used in model-verification:
                w.hhand.little.previous = hhand_q2cart.little(length_index + length_resolution * 2, :);
                little_z_of_m(angle_index) =   hhand_q2cart.little(15, z_axis);
                little_x_of_m(angle_index) =   hhand_q2cart.little(15, x_axis);
                little_y_of_m(angle_index) =   hhand_q2cart.little(15, y_axis);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
        
	
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        %THUMB FINGER
            hhand.thumb.prox.rt_Q    = hhand.thumb.prox.raster(angle_index);                  
			hhand.thumb.media.rt_Q   = hhand.thumb.media.raster(angle_index);
			hhand.thumb.distal.rt_Q  = hhand.thumb.distal.raster(angle_index);
			hhand.thumb.abduct.rt_Q  = hhand.thumb.abduct.raster(angle_index);
                
        % Now calculate the world points in cartesian space correspoing
        % to this posture:
        thumb_fwd;
        thumb_points;
            
            if(log)        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % the following may be used in model-verification:
                w.hhand.thumb.previous = hhand_q2cart.thumb(length_index + length_resolution * 2, :);
                thumb_z_of_m(angle_index) =   hhand_q2cart.thumb(15, z_axis);
                thumb_x_of_m(angle_index) =   hhand_q2cart.thumb(15, x_axis);
                thumb_y_of_m(angle_index) =   hhand_q2cart.thumb(15, y_axis);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
    case single_index_close
        
        if(angle_index > 128)
            angle_index = 1;
        else
            angle_index = angle_index + 1;
        end

        hhand.index.prox.rt_Q     = hhand.index.prox.raster(angle_index);                  
        hhand.index.media.rt_Q    = hhand.index.media.raster(angle_index);
        hhand.index.distal.rt_Q   = hhand.index.distal.raster(angle_index);
        hhand.index.abduct.rt_Q   = hhand.index.abduct.raster(angle_index);
        
        hhand.middle.prox.rt_Q    = hhand.index.prox.rt_Q / 2;                  
    	hhand.middle.media.rt_Q   = hhand.index.media.rt_Q / 4;
    	hhand.middle.distal.rt_Q  = hhand.index.distal.rt_Q / 8;
    	hhand.middle.abduct.rt_Q  = hhand.index.abduct.rt_Q / 4;
        
        hhand.ring.prox.rt_Q      = hhand.index.prox.rt_Q / 4;
    	hhand.ring.media.rt_Q     = hhand.index.media.rt_Q / 4;
    	hhand.ring.distal.rt_Q    = hhand.index.distal.rt_Q / 8;
    	hhand.ring.abduct.rt_Q    = 0;
        
        hhand.little.prox.rt_Q    = hhand.index.prox.rt_Q / 4;
    	hhand.little.media.rt_Q   = hhand.index.media.rt_Q / 4;
    	hhand.little.distal.rt_Q  = 0;
    	hhand.little.abduct.rt_Q  = 0;
        
        hhand.thumb.prox.rt_Q     = 0;                  
    	hhand.thumb.media.rt_Q    = 0;
    	hhand.thumb.distal.rt_Q   = 0;
    	hhand.thumb.abduct.rt_Q   = 0;
        
        % Now calculate the world points in cartesian space corresponding
        % to this posture:
        index_fwd;
        index_points;
        middle_fwd;
        middle_points;
        
        ring_fwd;
        ring_points;
        
        little_fwd;
        little_points;
        
        thumb_fwd;
        thumb_points;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
%     case random_move
%         
%         if(new_start)
%             ref = angle_resolution * fix(rand(1));
%             new_start = 0;
%         else
%             ref = ref + 150 * fix(rand(1));
%         end
%         
%         ind_index = ref + 50 * fix(rand(1));
%         if(ind_index < 1)
%             ind_index = 1;
%         end
%         if(ind_index > angle_resolution)
%             ind_index = angle_resolution;
%         end
%         hhand.index.prox.rt_Q     = hhand.index.prox.raster(ind_index);                  
%         hhand.index.media.rt_Q    = hhand.index.media.raster(ind_index);
%         hhand.index.distal.rt_Q   = hhand.index.distal.raster(ind_index);
%         hhand.index.abduct.rt_Q   = hhand.index.abduct.raster(ind_index);
%         
%         mid_index = ref + 50 * fix(rand(1));
%         if(mid_index < 1)
%             mid_index = 1;
%         end
%         if(mid_index > angle_resolution)
%             mid_index = angle_resolution;
%         end
%         hhand.middle.prox.rt_Q    = hhand.index.prox.raster(mid_index);                  
%     	hhand.middle.media.rt_Q   = hhand.index.media.raster(mid_index);
%     	hhand.middle.distal.rt_Q  = hhand.index.distal.raster(mid_index);
%     	hhand.middle.abduct.rt_Q  = hhand.index.abduct.raster(mid_index);
%         
%         ring_index = ref + 50 * fix(rand(1));
%         if(ring_index < 1)
%             ring_index = 1;
%         end
%         if(ring_index > angle_resolution)
%             ring_index = angle_resolution;
%         end
%         hhand.ring.prox.rt_Q      = hhand.index.prox.raster(ring_index);
%     	hhand.ring.media.rt_Q     = hhand.index.media.raster(ring_index);
%     	hhand.ring.distal.rt_Q    = hhand.index.distal.raster(ring_index);
%     	hhand.ring.abduct.rt_Q    = hhand.index.abduct.raster(ring_index);
%         
%         little_index = ref + 50 * fix(rand(1));
%         if(little_index < 1)
%             little_index = 1;
%         end
%         if(little_index > angle_resolution)
%             little_index = angle_resolution;
%         end
%         hhand.little.prox.rt_Q    = hhand.index.prox.raster(little_index);
%     	hhand.little.media.rt_Q   = hhand.index.media.raster(little_index);
%     	hhand.little.distal.rt_Q  = hhand.index.distal.raster(little_index);
%     	hhand.little.abduct.rt_Q  = hhand.index.abduct.raster(little_index);
%         
%         thumb_index = ref + 50 * fix(rand(1));
%         if(thumb_index < 1)
%             thumb_index = 1;
%         end
%         if(thumb_index > angle_resolution)
%             thumb_index = angle_resolution;
%         end
%         hhand.thumb.prox.rt_Q     = hhand.index.prox.raster(thumb_index);
%     	hhand.thumb.media.rt_Q    = hhand.index.media.raster(thumb_index);
%     	hhand.thumb.distal.rt_Q   = hhand.index.distal.raster(thumb_index);
%     	hhand.thumb.abduct.rt_Q   = hhand.index.abduct.raster(thumb_index);
%         
%         % Now calculate the world points in cartesian space corresponding
%         % to this posture:
%         index_fwd;
%         index_points;
%         middle_fwd;
%         middle_points;
%         
%         ring_fwd;
%         ring_points;
%         
%         little_fwd;
%         little_points;
%         
%         thumb_fwd;
%         thumb_points;
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end