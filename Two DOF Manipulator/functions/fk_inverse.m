function [ theta_1, theta_2 ] = fk_inverse( x, y, L_1, L_2)
theta_2 = pi - acos((L_1^2 + L_2^2 -x^2 -y^2)\(2*L_1*L_2));
theta_1 = atan(y/x) - atan((L_2*sin(theta_2))\(L_1 +L_2*cos(theta_2)))
end

