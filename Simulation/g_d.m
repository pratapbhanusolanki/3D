%computes the derivative of the gaussian 
function y = g_d(x)
   a1 = 0.7979;
    b1 =-0.7089;
    c1 = 8.257;
    a2 = 0.2016;
    b2 =-1.076;
    c2 = 29.78;
   
   y = -2*a1*((x-b1)/c1^2)*exp(-((x-b1)/c1)^2) -2*a2*((x-b2)/c2^2)*exp(-((x-b2)/c2)^2);
end