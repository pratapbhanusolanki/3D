clear all;
close all;
clc;

for i=-180:1:180
    x(i+181) = i;
    gauss(i+181) = gaussian_value(i);
    gauss_der(i+181) = gaussian_derivative(i);
end


subplot(2,1,1);
plot(x,gauss);

subplot(2,1,2);
plot(x,gauss_der);