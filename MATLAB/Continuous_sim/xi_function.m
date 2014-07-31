function [ val ] = xi_function( d_up,d_down,d,f1,f2 )
%XI_FUNCTION Interpolates the functions f1 and f2

    val = 0;

    if d <= d_down
        
        val = f1;
        
    else if d > d_down && d <= d_up
            
            val = (d_up-d)/(d_up-d_down)*f1+(d-d_down)/(d_up-d_down)*f2;
            
        else if d > d_up
                
                val = f2;
                
            end
            
        end
        
    end
 

end

