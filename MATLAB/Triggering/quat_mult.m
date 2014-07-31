function [ q_o ] = quat_mult( q_1, q_2 )
%QUAT_MULT Multiplies two quaternions, assuming scalar part at the end

    W=[q_2(4) q_2(3) -q_2(2) q_2(1);
       -q_2(3) q_2(4) q_2(1) q_2(2);
       q_2(2) -q_2(1) q_2(4) q_2(3);
       -q_2(1) -q_2(2) -q_2(3) q_2(4)];
   
   q_o=W*q_1';
   q_o=q_o'./norm(q_o);

end

