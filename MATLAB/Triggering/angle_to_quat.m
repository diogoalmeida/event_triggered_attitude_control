function [ quat ] = angle_to_quat( Orientation)
%ANGLE_TO_QUAT Converts from a specified orientation to a quaternion,
%assuming Orientation = [roll, pitch, yaw]. The resulting quaternion has
%the vectorial part before the scalar

    
    roll = Orientation(1);
    pitch = Orientation(2);
    yaw = Orientation(3);
    
    quat=zeros(1,4);
    
    quat(4) = cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    quat(1) = sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    quat(2) = cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    quat(3) = cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    


end

