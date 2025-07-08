function [T] = forward_kinematics(r1,r2,t1,t2)

T=[cosd(t1+t2) -sind(t1+t2) 0  r1*cosd(t1)+r2*cosd(t2+t1);
   sind(t1+t2)  cosd(t1+t2) 0  r1*sind(t1)+r2*sind(t2+t1);
   0 0 1 0; 
   0 0 0 1];

end