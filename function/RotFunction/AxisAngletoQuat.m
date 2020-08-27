
function q = AxisAngletoQuat(axis,angle)
    axis = axis/norm(axis);
    q = [cos(angle/2); sin(angle/2)*axis(1,1); sin(angle/2)*axis(2,1); sin(angle/2)*axis(3,1)];
    if angle > pi
        q = -q;
    end
end
