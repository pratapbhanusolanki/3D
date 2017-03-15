clear all;
close all;
clc;

x=0.03*pi:0.03*pi:2*pi;

y = (255*sin(x)+255)/2;
z = floor(y);
plot(z);
grid on;