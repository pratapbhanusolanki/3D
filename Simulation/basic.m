%Simulates the 3D tracking of the beam 
clear all;
close all;
clc;

x(:,1) = [4,4,5]';
x_hat_k(:,1) = [3,0,0]';

gamma(1) = acosd(cosd(x_hat_k(2,1)/18)*cosd(x_hat_k(3,1)));
y_hat_series(1) = x_hat_k(1,1)*gaussian_value(gamma(1));


%Noise Covariance Matrices and Kalman filter parameters
Q_system = 0.01*[1,0,0;0,18,0;0,0,1];
Q = 0.5*[1,0,0;0,700,0;0,0,50;];
R = eye(3);
R_inv = inv(R);
R_system = 0.1*eye(2);

P(:,:,1) = 100*[1,0,0;0,180,0;0,0,10];
P_current = P(:,:,1);

%System Matrices
A = eye(3);
B = [0,0;1,0;0,1];

%Scanning parameters
angle_bias(1) = 0;
phi = 45;
scan_radius = 10;


%Number of iterations, it changes the length of simulation
num_el = 800;

theta(1) = 45;
psi(1) = 0;


%Previous values needed for initialisation
u2_previous = 0.1;
u3_previous = 0.1;

u2_k = 0.1;
u3_k = 0.1;

u2 = [u2_k, u2_previous];
u3 = [u3_k, u3_previous];

previous_measurement = 0;
previous_previous_measurement = 0;

diff_sum = 0;
previous_difference = 0;
normal_u2 = 0;
normal_u3 = 0;


num_iteration = 800;

for i=2:num_iteration
	tic;
	x(:,i) = x(:,i-1)+ [0; normal_u2;normal_u3] + 0.0*[0; 1;1/18] + [normrnd(0,Q_system(1,1)); normrnd(0,Q_system(2,2));normrnd(0,Q_system(3,3))];
	theta(i) = theta(i-1) + u3_k;
    
    
	x1_hat_k = x_hat_k(1,i-1);
    x2_hat_k = x_hat_k(2,i-1);
    x3_hat_k = x_hat_k(3,i-1);
    
    x1 = x(1,i);
    x2 = x(2,i);
    x3 = x(3,i);

    angle_bias(i) = angle_bias(i-1) + phi;
    bias = angle_bias(i);
	previous_alpha_bias = scan_radius*sind(bias-phi);
	previous_beta_bias = scan_radius*cosd(bias-phi);
	alpha_bias = scan_radius*sind(bias);
	beta_bias = scan_radius*cosd(bias);
    
    alpha_diff = alpha_bias - previous_alpha_bias;
    beta_diff = beta_bias - previous_beta_bias; 
    
    C = get_C_matrix(x_hat_k(:,i-1), u2, u3,scan_radius, bias, phi);
    rank_C(i) = rank(C);
    rank(C);
    P_current = A*P_current*A' + Q;
    
    % Output calculation
    measurement = x1*gaussian_value(acosd(cosd(x2/18 + beta_bias)*cosd(x3 + alpha_bias))) + normrnd(0,R_system(1,1));
    y = [measurement;previous_measurement; previous_previous_measurement];
    y_hat = get_output_array(x_hat_k(:,i-1), u2, u3,scan_radius, bias, phi);
    y_hat_series(i) = y_hat(1); 
    y_series(i) = measurement;
    
        
    % Filtering    
    K = P_current*C'*inv(C*P_current*C' + R);
    x_hat_k(:,i) = x_hat_k(:,i-1)+K*(y-y_hat);
    P(:,:,i) = (eye(3) - K*C)*P_current;
    P_current = P(:,:,i);

    difference = abs(y(1)-y_hat(1));
    diff_sum = diff_sum + difference;
    
    if x_hat_k(1,i) < 0
        x_hat_k(1,i) = 0;
    end
    
    if(difference + previous_difference < 0.5)
        G = 0.5;
        G2 = 0.1;
    else
        G = 0.5;
        G2 = 0.1;
    end
    previous_difference = difference;
    
    normal_u2 = -G*x_hat_k(2,i);
    normal_u3 = -G*x_hat_k(3,i);
    
    alpha_u = -G*x_hat_k(3,i) + alpha_diff;
    beta_u = -G*x_hat_k(2,i) + beta_diff;

    u2_k = atan2d(cosd(alpha_u)*sind(beta_u), cosd(alpha_u)*cosd(beta_u)*cosd(theta(i)) - sind(alpha_u)*cosd(theta(i)));
    u3_k = asind(cosd(alpha_u)*cosd(beta_u)*sind(theta(i)) + sind(alpha_u)*cosd(theta(i)));

    u2 = [normal_u2, u2_previous];
    u3 = [normal_u2, u3_previous];

    u2_previous = normal_u2;
    u3_previous = normal_u3;
    
    x_hat_k(:,i) = x_hat_k(:,i)+ [0; normal_u2;normal_u3];
    
    theta(i) = theta(i-1) + u3_k;
    psi(i) = psi(i-1) + u_2k;
    
    x(i) = cos(theta(i))*cos(psi(i));
    y(i) = cos(theta(i))*sin(psi(i));
    z(i) = (theta(i));

    t(i) = toc;
end


T = 0.08;
time = T:T:num_iteration*T;


subplot(4,1,1);
%plot(time,x(1,:));
hold on;
%plot(x_hat_data(1,1:num_iteration),'r');
plot(time,x(1,:),'r');
plot(time,x_hat_k(1,:));
%title('Convergence');
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_1$$','Interpreter','Latex');
legend('Original state','Extended Kalman Filter')

subplot(4,1,2);
hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
plot(time,x(2,:),'r');
plot(time,x_hat_k(2,:));
%ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_2$$','Interpreter','Latex');

subplot(4,1,3);

hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
plot(time,x(3,:),'r');
plot(time,x_hat_k(3,:));
%ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{x}_3$$','Interpreter','Latex');

subplot(4,1,4);
hold on;
%plot(time,x(2,:));
%plot(y_hat_data(1,1:num_iteration),'r');

plot(time, y_series(1,1:num_iteration),'r');
%plot(time, measurement_data,'k');
plot(time, y_hat_series,'b')
legend('Simulated measurement','Estimated measurement')

%ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{y}$$','Interpreter','Latex');
ylim([0,5]);

%figure;
dummy_x = -50:0.5:50;
dummy_y = gaussian_value(dummy_x);
%plot(dummy_x,dummy_y);

figure;
plot3(x,y,z);