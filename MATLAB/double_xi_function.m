function [ val ] = double_xi_function(d_3,d_4,d_1,d_2,d,f1,f2)
%DOUBLE_XI Interpolates between f1 and f2 and back to f1

    val_1 = xi_function(d_4,d_3,d,f2,f1);
    
    val = xi_function(d_2,d_1,d,f1,val_1);
    

end

