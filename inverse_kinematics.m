function [t1, t2] = inverse_kinematics(r1,r2, px, py)

cost2= (px^2 + py^2 - r1^2 - r2^2) / (2 * r1 * r2);
t2 = acosd(cost2)
y1 = r2 * sind(t2);
x1 = r1 + r2 * cosd(t2);
a1 = py/px;
a2 = y1/x1;
b1 = atand(a1);
b2 = atand(a2);
t1 = atand(a1) - atand(a2)

end