function [ val ] = delta_integral(d_up, d_low, d)
%DELTA_INTEGRAL Computes the integral from 0 to d of the delta function
%defined in the fast and saturating attitude controller paper

    val = 0;
    
    if d >= 0 && d <= d_low
        
        val = d^2/2;
        
    else if d > d_low && d <= d_up
            
            val = d_low^2/2 + d_low*(d-d_low);
            
        else if d > d_up && d <= pi
                
                val = d_low^2/2 + d_low*(d_up-d_low) + d_low*((d^2-d_up^2)/(2*(d_up-pi))-pi*(d-d_up)/(d_up-pi));
                
            end
            
        end
        
    end
        

end

