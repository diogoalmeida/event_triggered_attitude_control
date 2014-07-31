function [ val ] = delta_function( d_up, d_low, d)
%DELTA_FUNCTION Implements the delta function defined in the saturating
%thrust controller paper.

    val = 0;
    
    if d >= 0 && d <= d_low
        
        val = d;
        
    else if d > d_low && d <= d_up
            
            val = d_low;
            
        else if d > d_up && d <= pi
                
                val = d_low * (d-pi)/(d_up - pi);

            end
            
        end
        
    end

end

