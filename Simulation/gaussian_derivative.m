function y = gaussian_derivative(x)
   a1 =      0.6922/1.0359;
   b1 =       7.752;
   c1 =     148.8;
   a2 =       0.346/1.0359;
   b2 =      -13.57;
   c2 =       325.8; 
   x = x*18;
   
   y = -2*a1*((x-b1)/c1^2)*exp(-((x-b1)/c1)^2) -2*a2*((x-b2)/c2^2)*exp(-((x-b2)/c2)^2);
   y = 18*y;
end
