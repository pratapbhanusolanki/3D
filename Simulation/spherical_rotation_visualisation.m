clear all;
close all;
clc;

[X,Y] = meshgrid(-1:0.01:1);
r_source = 1.2;
Z = real(sqrt(1-X.*X - Y.*Y));
hFig = figure;
%set(hFig, 'Position', [680 678 1400 1050])
surf(X,Y,real(Z));