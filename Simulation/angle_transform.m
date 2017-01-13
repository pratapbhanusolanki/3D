function y = angle_transform(alpha,beta,theta)
%transform the local coordinates alpha, beta into global coordinates, when
%the local coordinate is at elevation theta
    beta_prime = atan2d(cosd(alpha)*sind(beta), cosd(alpha)*cosd(beta)*cosd(theta) - sind(alpha)*sind(theta));
    alpha_prime = asind(cosd(alpha)*cosd(beta)*sind(theta) + sind(alpha)*cosd(theta))-theta;
    y = [beta_prime;alpha_prime];
end