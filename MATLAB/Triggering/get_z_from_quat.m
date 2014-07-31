function [ q_z ] = get_z_from_quat( q )
%GET_Z_FROM_QUAT Given a quaternion, it returns its component of rotation
%around the z axis

    q_z=zeros(1,4);
    q_z(3)=q(3)/(sqrt(q(3)^2+q(4)^2));
    q_z(4)=q(4)/(sqrt(q(3)^2+q(4)^2));

    q_z = q_z./norm(q_z);
end

