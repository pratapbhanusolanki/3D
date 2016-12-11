%approximation testing
clear all;
close all;
clc;

[X,Y] = meshgrid(-90:1:90);

Z1 = exact_measurement_model(1,X,Y);
Z2 = approx_measurement_model(1,X,Y);

surf(X,Y,Z1-Z2);

hold on;
surf(X,Y,Z1);

x = -90:1:90;
y = 5;

z1 = exact_measurement_model(1,x,y);
z2 = approx_measurement_model(1,x,y);

figure;
plot(x,z1,'r');
hold on;
plot(x,z2,'b');
plot(x,z1-z2,'g');

legend('exact model', 'approximate model', 'difference');
xlabel('\theta');
ylabel('Gaussian function values');
title('plot of the two dimensional gaussian fitting functions, with y = 5')


