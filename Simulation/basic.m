%Simulates the 3D tracking of the beam 
clear all;
close all;
clc;

x(:,1) = [4,-360,30]';
x_hat_k(:,1) = [3,0,0]';

gamma(1) = acosd(cosd(x(1,2)/18)*cosd(x(1,3)));
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
bias_radius = 6;


%Number of iterations, it changes the length of simulation
num_el = 800;


for i=2:num_iteration
	tic;
	x(:,i) = x(:,i-1)+ [0;u2;u3] + 0.5*[0; 1;1/18] + [normrnd(0,Q_system(1,1)); normrnd(0,Q_system(2,2));normrnd(0,Q_system(3,3))];
	
	x1_hat_k = x_hat_k(1,i-1);
    x2_hat_k = x_hat_k(2,i-1);
    x3_hat_k = x_hat_k(3,i-1);

    angle_bias(i) = angle_bias(i-1) + phi;
    bias = angle_bias(i-1);
	previous_alpha_bias = scan_radius*cosd(bias-phi);
	previous_beta_bias = scan_radius*cosd(bias-phi);
	alpha_bias = scan_radius*cosd(bias);
	beta_bias = scan_radius*sind(bias);
    alpha_bias = cosd(angle_bias(i));
    beta_bias = sind(angle_bias(i));

    C = get_C_matrix(x_hat_k(:,i-1),m_command2,m_command3);
    y = get_output_array(x_hat_k(:,i-1),m_command2,m_command3);







end