clear all;
close all;
clc;

load 'data/large_INI_with_control.mat'
figure;
surf(X(1:end,:),Y(1:end,:),p_norm_mesh_mean_all(1:end,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('||P||_2');

figure;
surf(X(1:end,:),Y(1:end,:),control_effort_all(1:end,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('U_n');

load 'data/large_INI_without_control.mat'

figure;
surf(X(1:end-1,:),Y(1:end-1,:),p_norm_mesh_mean_all(1:end-1,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('||P||_2');

figure;
surf(X(1:end-1,:),Y(1:end-1,:),control_effort_all(1:end-1,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('U_n');

load 'data/small_INI_with_control.mat'
figure;
surf(X(1:end-1,:),Y(1:end-1,:),p_norm_mesh_mean_all(1:end-1,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('||P||_2');

figure;
surf(X(1:end-1,:),Y(1:end-1,:),control_effort_all(1:end-1,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('U_n');

load 'data/small_INI_without_control.mat'

figure;
surf(X(1:end-1,:),Y(1:end-1,:),p_norm_mesh_mean_all(1:end-1,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('||P||_2');

figure;
surf(X(1:end-1,:),Y(1:end-1,:),control_effort_all(1:end-1,:));
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('U_n');