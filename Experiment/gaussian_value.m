function y = gaussian_value2(x)
       a1 =    0.6972;   %  0.7113 ;
       b1 =    -0.04401;
       c1 =       10.11;
       a2 =     0.3028;% 0.3089;
       b2 =     0.05681;
       c2 =       25.19;
    y =  a1*exp(-((x-b1)/c1)^2) + a2*exp(-((x-b2)/c2)^2)
    
end