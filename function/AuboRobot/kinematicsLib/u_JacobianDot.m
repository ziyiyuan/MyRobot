function Jdot = JacobianDot(z_axis,origin,qd)
%input current q_dot
w = [[]];
z_dot = [[]];
d_dot = [[]];
zd_dot = [[]];
for i = 1:1:6
    if (i==1)
        w{i} = qd(i)*z_axis{i};
        d_dot{i} = [0;0;0];
        z_dot{i} = [0;0;0];
    else
        w{i} = w{i-1} + qd(i)*z_axis{i};
        d_dot{i} = d_dot{i-1} + cross(w{i-1},origin{i}- origin{i-1});
        z_dot{i} = cross(w{i-1},z_axis{i});
    end   
end

Jdot = [];
for i = 1:1:6
    zd_dot{i} = cross(z_dot{i},origin{6}-origin{i}) + cross(z_axis{i},d_dot{6}-d_dot{i});
    c{i} = [zd_dot{i};z_dot{i}];
    Jdot = [Jdot,c{i}];
end
end
