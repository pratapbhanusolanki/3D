%Uses a simpler model g(x,y) = g(x)*g(y) in the simulation 

clear all;
close all;
clc;

x_hat_k(:,1) = [3,0,0]';

y_hat_series(1) = 0;


%Noise Covariance Matrices and Kalman filter parameters
Q = 1*[1,0,0;0,1,0;0,0,1;];
R = eye(3);
R_inv = inv(R);

P(:,:,1) = [1,0,0;0,1,0;0,0,1];
P_current = P(:,:,1);

%System Matrices
A = eye(3);
B = [0,0;1,0;0,1];

%Scanning parameters
angle_bias(1) = 0;
phi = 20;
scan_radius = 5;

%Initial position parameters
theta(1) = 0;
psi(1) = 0;

scan_theta(1) = 45;
scan_psi(1) = -110;

%Previous values needed for initialisation
u2_previous = -1.0;
u3_previous = -2.0;

u2_k = 0.0;
u3_k = 0.0;

u2 = [u2_k; u2_previous];
u3 = [u3_k; u3_previous];

previous_measurement = 2;
previous_previous_measurement = 2;

diff_sum = 0;
previous_difference = 0;
normal_u2 = 0;
normal_u3 = 0;





%Reading all the data
unzip('data.npz','data')
x_hatf_data = readNPY('data/x_hatf_all.npy');
x_hat_data = readNPY('data/x_hat_all.npy');
Pf_data = permute(readNPY('data/Pf_all.npy'),[2,3,1]);
P_data = permute(readNPY('data/P_all.npy'),[2,3,1]);
y_data = readNPY('data/y_all.npy');
y_hat_data = readNPY('data/y_hat_all.npy');
C_data = permute(readNPY('data/C_all.npy'),[2,3,1]);
K_data = permute(readNPY('data/K_all.npy'),[2,3,1]);
num_iteration = length(x_hat_data);

for i=2:num_iteration
    
    %set(hFig, 'Position', [680 678 1400 1050])
    expInd = i-1
	tic;
    x_hat_k(:,i) = x_hat_k(:,i-1) + [0;normal_u2; normal_u3];
    dumS = x_hat_k(:,i)'
    dumE = x_hatf_data(i,:)

	x1_hat_k = x_hat_k(1,i);
    x2_hat_k = x_hat_k(2,i);
    x3_hat_k = x_hat_k(3,i);
    
    angle_bias(i) = angle_bias(i-1) + phi;
    bias = angle_bias(i);
	previous_alpha_bias = scan_radius*sind(bias-phi);
	previous_beta_bias = scan_radius*cosd(bias-phi);
	alpha_bias = scan_radius*sind(bias);
	beta_bias = scan_radius*cosd(bias);
    
    alpha_diff = alpha_bias - previous_alpha_bias;
    beta_diff = beta_bias - previous_beta_bias; 
    
    previous_u = [u2,u3];
    scan_parameters = [scan_radius, bias, phi];
    
    C = get_C_matrix(x_hat_k(:,i),previous_u,scan_parameters)
    dum = C_data(:,:,i)
    rank_C(i) = rank(C);
    rank(C);
    P_current = A*P_current*A' + Q
    dum = Pf_data(:,:,i)
    
    
    % Output calculation
    measurement = y_data(i,1);
    y = [measurement;previous_measurement; previous_previous_measurement]
    dum = y_data(i,:)'
    
    y_hat = get_output_array(x_hat_k(:,i-1), previous_u,scan_parameters)
    dum = y_hat_data(i,:)'
    
    y_hat_series(i) = y_hat(1);
    y_series(i) = measurement;
    
    previous_previous_measurement = previous_measurement;
    previous_measurement = measurement;
    
    
    % Filtering    
    K = P_current*C'*inv(C*P_current*C' + R)
    dum = K_data(:,:,i)
    
    x_hat_k(:,i) = x_hat_k(:,i-1)+K*(y-y_hat);
    dumS = x_hat_k(:,i)'
    dumE = x_hat_data(i,:)
    P(:,:,i) = (eye(3) - K*C)*P_current;
    P_current = P(:,:,i)
    dum = P_data(:,:,i)

    difference = abs(y(1)-y_hat(1));
    diff_sum = diff_sum + difference;
    
    if x_hat_k(1,i) < 0
        x_hat_k(1,i) = 0;
    end
    
    if(difference + previous_difference < 2)
        G = 0.2;
        G2 = 0.2;
    else
        G = 0.1;
        G2 = 0;
    end
    
    G = 0.0;
    previous_difference = difference;
    
    normal_u2 = -G*x_hat_k(2,i);
    normal_u3 = -G*x_hat_k(3,i);
    
    u2 = [normal_u2; u2_previous];
    u3 = [normal_u3; u3_previous];

    u2_previous = normal_u2;
    u3_previous = normal_u3;
    [dummy_u3,u2_k] = angle_transform(normal_u3, normal_u2, theta(i-1));
    u3_k = dummy_u3 - theta(i-1);
    
    psi(i) = psi(i-1) + u2_k;
    theta(i) = theta(i-1) + u3_k;  
    
    %Computations related to plotting and motor commands
    [theta_offset_temp,psi_offset] = angle_transform(alpha_bias, beta_bias, theta(i));
    theta_offset = theta_offset_temp-theta(i);
   
    scan_psi(i) = psi(i) + psi_offset;
    scan_theta(i) = theta(i) + theta_offset;
    
    Motor_command_psi = scan_psi(i) - scan_psi(i-1);
    Motor_command_theta = scan_theta(i) - scan_theta(i-1);
end


T = 0.08;
time = T:T:num_iteration*T;

T = 0.08;
time = T:T:num_iteration*T;



figure;

subplot(4,1,1);
%plot(time,x(1,:));
hold on;
%plot(x_hat_data(1,1:num_iteration),'r');
%plot(time,x(1,:),'r');
plot(time,x_hat_data(:,1));
%title('Convergence');
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_1$$','Interpreter','Latex');
legend('Original state','Extended Kalman Filter')

subplot(4,1,2);
hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
%plot(time,x(2,:),'r');
plot(time,x_hat_data(:,2));
%ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_2$$','Interpreter','Latex');

subplot(4,1,3);

hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
%plot(time,x(3,:),'r');
plot(time,x_hat_data(:,3));
ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_3$$','Interpreter','Latex');

subplot(4,1,4);
hold on;
%plot(time,x(2,:));
%plot(y_hat_data(1,1:num_iteration),'r');

plot(time, y_series,'r');
%plot(time, measurement_data,'k');
plot(time, y_hat_series,'b')
legend('Experimental measurement','Estimated measurement')

ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{y}$$','Interpreter','Latex');
ylim([0,10]);




