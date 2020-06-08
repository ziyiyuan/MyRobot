function f = skewStar(w)
% use in dynamics
    f = [w(1) w(2) w(3) 0 0 0; 0 w(1) 0 w(2) w(3) 0; 0 0 w(1) 0 w(2) w(3)];
end