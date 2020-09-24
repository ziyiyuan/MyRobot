function Jdot = cal_jacobian_dot(q,qd,DHMatrix)
% to do! _ziyi
T0 = forward_kinematics(q, DHMatrix)
z_axis = [[]]; origin = [[]];
for  i = 1:1:6
    z_axis{i} = T0{i}(1:3,3); % Ðý×ªÖá in base
    origin{i} = T0{i}(1:3,4); % Î»ÖÃ
end

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

c = [[]];
Jdot = [];
for i = 1:1:6
    zd_dot{i} = cross(z_dot{i},origin{6}-origin{i}) + cross(z_axis{i},d_dot{6}-d_dot{i});
    c{i} = [zd_dot{i};z_dot{i}];
    Jdot = [Jdot,c{i}];
end
end
