function R = AxisAngletoRotMatrix(axis,angle)
rx = axis(1);
ry = axis(2);
rz = axis(3);
ct = cos(angle);
st = sin(angle);
xct = rx*(1-ct);
yct = ry*(1-ct);
zct = rz*(1-ct);
xst = rx*st;
yst = ry*st;
zst = rz*st;

R = [rx*xct + ct, rx*yct - zst, rx*zct + yst
    ry*xct + zst, ry*yct + ct, ry*zct - xst
    rz*xct - yst, rz*yct + xst, rz*zct + ct];
end









