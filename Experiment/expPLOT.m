%Uses a simpler model g(x,y) = g(x)*g(y) in the simulation 

clear all;
close all;
clc;

x_hat_k(:,1) = [3,0,0]';

y_hat_series(1) = 0;


%Reading all the data
addpath('npy-matlab-master')
unzip('Data/data_2017-09-20_11:46:13_AM.npz','data')
x_hatf_data = readNPY('data/x_hatf_all.npy');
x_hat_data = readNPY('data/x_hat_all.npy');
Pf_data = permute(readNPY('data/Pf_all.npy'),[2,3,1]);
P_data = permute(readNPY('data/P_all.npy'),[2,3,1]);
previous_u_data = permute(readNPY('data/previous_u_all.npy'),[2,3,1]);
y_data = readNPY('data/y_all.npy');
y_hat_data = readNPY('data/y_hat_all.npy');
C_data = permute(readNPY('data/C_all.npy'),[2,3,1]);
K_data = permute(readNPY('data/K_all.npy'),[2,3,1]);
motor_data = readNPY('data/motor_commands_all.npy');
time = readNPY('data/timer.npy');
interval = readNPY('data/interval.npy');
disturbance_term = readNPY('data/disturbance_term.npy');
num_iteration = length(x_hat_data);


figure;
subplot(4,1,1);
%plot(time,x(1,:));
hold on;
%plot(x_hat_data(1,1:num_iteration),'r');
%plot(time,x(1,:),'r');
plot(time,x_hat_data(:,1));
%plot(time,x_hat_k(1,:));
%title('Convergence');
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_1$$','Interpreter','Latex');
%legend('Experimental Estimate','Simulation Estimate')


subplot(4,1,2);
hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
%plot(time,x(2,:),'r');
plot(time,x_hat_data(:,2));
%plot(time,x_hat_k(2,:));
%ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_2$$','Interpreter','Latex');
%legend('Experimental Estimate','Simulation Estimate')


subplot(4,1,3);
hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
%plot(time,x(3,:),'r');
plot(time,x_hat_data(:,3));
%plot(time,x_hat_k(3,:));
%ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_3$$','Interpreter','Latex');
%legend('Experimental Estimate','Simulation Estimate')

subplot(4,1,4);
hold on;
%plot(time,x(2,:));
%plot(y_hat_data(1,1:num_iteration),'r');
plot(time, y_data(:,1),'r');
%plot(time, measurement_data,'k');
%plot(time, y_hat_series,'b');
plot(time,y_hat_data(:,1));
legend('Actual Measurement','EKF Estimate')

%ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{y}$$','Interpreter','Latex');
%ylim([0,10]);




