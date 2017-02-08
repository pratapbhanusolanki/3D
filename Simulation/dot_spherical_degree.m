function y = dot_spherical_degree(A,B)
    r1 = A(1);
    phi1 = A(2);
    theta1 = A(3);
    
    r2 = B(1);
    phi2 = B(2);
    theta2 = B(3);
    
    dot_angle = cosd(phi1)*cosd(theta1)*cosd(phi2)*cosd(theta2)+...
                sind(phi1)*cosd(theta1)*sind(phi2)*cosd(theta2)+...
                sind(theta1)*sind(theta2);
    y = r1*r2*dot_angle;
end