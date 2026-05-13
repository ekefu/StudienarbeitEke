% find a linear region of operation

load('nonspread_rwl');

smooth_part_only = 1;

if(smooth_part_only)
    
    for i=10001:20000
        smooth_non_x_of_m(i-10000) = non_x_of_m(i);
        smooth_non_z_of_m(i-10000) = non_z_of_m(i);
    end
    
    %do a least squares function approximation.
    
    derivatives = 0;
    curve = 1;

    if(derivatives)
        subplot(2,3,1),plot(smooth_non_x_of_m)
		xlabel('x nonspread')
		
		subplot(2,3,4),plot(smooth_non_z_of_m)
		xlabel('z nonspread')
		
		subplot(2,3,2),plot(diff(smooth_non_x_of_m))
		xlabel('diff1 x nonspread')
		
		subplot(2,3,5),plot(diff(smooth_non_z_of_m))
		xlabel('diff1 z nonspread')
		
		subplot(2,3,3),plot(diff(diff(smooth_non_x_of_m)))
		xlabel('diff2 x nonspread')
		
		subplot(2,3,6),plot(diff(diff(smooth_non_z_of_m)))
		xlabel('diff2 z nonspread')
    end
     
    
    if(curve)
        plot(smooth_non_x_of_m,smooth_non_z_of_m)
        axis([-70 100 -10 100])
        grid on
    end
    
    
else   
        
	subplot(2,3,1),plot(non_x_of_m)
	xlabel('x nonspread')
	
	subplot(2,3,4),plot(non_z_of_m)
	xlabel('z nonspread')
	
	subplot(2,3,2),plot(diff(non_x_of_m))
	xlabel('diff1 x nonspread')
	
	subplot(2,3,5),plot(diff(non_z_of_m))
	xlabel('diff1 z nonspread')
	
	subplot(2,3,3),plot(diff(diff(non_x_of_m)))
	xlabel('diff2 x nonspread')
	
	subplot(2,3,6),plot(diff(diff(non_z_of_m)))
	xlabel('diff2 z nonspread')% 
end