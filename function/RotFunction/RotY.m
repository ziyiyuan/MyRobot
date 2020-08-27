%pitch ¸©Ñö½Ç
function R = RotY(pitch)
ct = cos(pitch);
st = sin(pitch);
R = [
    ct   0   st
    0    1   0
    -st  0   ct
    ];
end