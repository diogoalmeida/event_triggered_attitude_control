function [ q_o ] = quat_mult_inv( q_1, q_2 )
%QUAT_MULT Performs W^-1(q_1)*q_2

    W=[q_1(4) q_1(3) -q_1(2) q_1(1);
       -q_1(3) q_1(4) q_1(1) q_1(2);
       q_1(2) -q_1(1) q_1(4) q_1(3);
       -q_1(1) -q_1(2) -q_1(3) q_1(4)];
   
   q_o=W\q_2';
   q_o=q_o';
   
   q_o = q_o./norm(q_o);
   
 
end

