function [ q_o ] = quat_conjugate( q_i )
%QUAT_CONJUGATE Returns the quaternion conjugate. Assumes quaternions with
%scalar parte as the last value


    q_o=zeros(1,4);
    q_o(1:3)=-q_i(1:3);
    q_o(4)=q_i(4);

end

