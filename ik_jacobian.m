function J = ik_jacobian(r1,r2,t1,t2)

 J = [-r1*sind(t1)-r2*sind(t1+t2),-r2*sind(t1+t2);
        r1*cosd(t1)+r2*cosd(t1+t2), r2*cosd(t1+t2)];

end

