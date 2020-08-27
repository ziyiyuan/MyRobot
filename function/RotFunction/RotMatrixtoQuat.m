 function q = RotMatrixtoQuat(t)

    qs = sqrt(trace(t)+1)/2.0; % w
    kx = t(3,2) - t(2,3);   % 4*w*x
    ky = t(1,3) - t(3,1);   % 4wy
    kz = t(2,1) - t(1,2);   % 4wz

    if (t(1,1) >= t(2,2)) && (t(1,1) >= t(3,3)) 
        kx1 = t(1,1) - t(2,2) - t(3,3) + 1; % Nx - Oy - Az + 1  == 4xx
        ky1 = t(2,1) + t(1,2);          % Ny + Ox == 4xy
        kz1 = t(3,1) + t(1,3);          % Nz + Ax  == 4xz
        add = (kx >= 0);
    elseif (t(2,2) >= t(3,3))
        kx1 = t(2,1) + t(1,2);          % Ny + Ox
        ky1 = t(2,2) - t(1,1) - t(3,3) + 1; % Oy - Nx - Az + 1
        kz1 = t(3,2) + t(2,3);          % Oz + Ay
        add = (ky >= 0);
    else
        kx1 = t(3,1) + t(1,3);          % Nz + Ax
        ky1 = t(3,2) + t(2,3);          % Oz + Ay
        kz1 = t(3,3) - t(1,1) - t(2,2) + 1; % Az - Nx - Oy + 1
        add = (kz >= 0);
    end

    if add
        kx = kx + kx1;
        ky = ky + ky1;
        kz = kz + kz1;
    else
        kx = kx - kx1;
        ky = ky - ky1;
        kz = kz - kz1;
    end
    nm = norm([kx ky kz]);
    if nm == 0
        q = Quaternion([1 0 0 0]);
    else
        s = sqrt(1 - qs^2) / nm;
        qv = s*[kx ky kz];
        
        q = [qs qv]';

    end
end