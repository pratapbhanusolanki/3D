%computes the derivative of the gaussian 
function y = g_d(x)
   a1 =    0.6972;   %  0.7113 ;
       b1 =    -0.04401;
       c1 =       10.11;
       a2 =     0.3028;% 0.3089;
       b2 =     0.05681;
       c2 =       25.19;
   
   y = -2*a1*((x-b1)/c1^2)*exp(-((x-b1)/c1)^2) -2*a2*((x-b2)/c2^2)*exp(-((x-b2)/c2)^2);
end