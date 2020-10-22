function IMatrix = inertiaMatrix(I)
IMatrix = [I(1),  I(2), I(3); I(2), I(4), I(5); I(3), I(5), I(6)];
end