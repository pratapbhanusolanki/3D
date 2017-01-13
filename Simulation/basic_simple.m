%Uses a simpler model g(x,y) = g(x)*g(y) in the simulation 

clear all;
close all;
clc;

x(:,1) = [4,20,20]';
x_hat_k(:,1) = [3,40,40]';

y_hat_series(1) = 0;


%Noise Covariance Matrices and Kalman filter parameters
Q_system = 0.01*[1,0,0;0,10,0;0,0,10];
Q = 0.1*[1,0,0;0,10,0;0,0,10;];
R = eye(3);
R_inv = inv(R);
R_system = 0.1*eye(2);

P(:,:,1) = [1,0,0;0,10,0;0,0,10];
P_current = P(:,:,1);

%System Matrices
A = eye(3);
B = [0,0;1,0;0,1];

%Scanning parameters
angle_bias(1) = 0;
phi = 20;
scan_radius = 10;


%Initial position parameters
theta(1) = 45;
psi(1) = -120;

scan_theta(1) = 45;
scan_psi(1) = -120;

%Previous values needed for initialisation
u2_previous = 0.1;
u3_previous = 0.1;

u2_k = 0.0;
u3_k = 0.0;

u2 = [u2_k; u2_previous];
u3 = [u3_k; u3_previous];

previous_measurement = 0;
previous_previous_measurement = 0;

diff_sum = 0;
previous_difference = 0;
normal_u2 = 0;
normal_u3 = 0;


num_iteration = 400;

[X,Y] = meshgrid(-1:0.01:1);
r_source = 1.2;
Z = real(sqrt(1-X.*X - Y.*Y));
hFig = figure;
%set(hFig, 'Position', [680 678 1400 1050])
surf(X,Y,real(Z));
frame(1) = getframe;
hold on;
for i=2:num_iteration
    
    %set(hFig, 'Position', [680 678 1400 1050])
    surf(X,Y,real(Z));
    hold on;
    axis(0.6*[-2 2 -2 2 -2 2])
    i
	tic;
	x(:,i) = x(:,i-1)+ [0; normal_u2;normal_u3] + 0.0*[0; 1;1/18] + [normrnd(0,Q_system(1,1)); normrnd(0,Q_system(2,2));normrnd(0,Q_system(3,3))];
	%theta(i) = theta(i-1) + u3_k;
    
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
    
    previous_u = [u2,u3];
    scan_parameters = [scan_radius, bias, phi];
    
    C = get_C_matrix(x_hat_k(:,i-1),previous_u,scan_parameters);
    rank_C(i) = rank(C);
    rank(C);
    P_current = A*P_current*A' + Q;
    
    % Output calculation
    measurement = exact_measurement_model(x1,x2 + beta_bias,x3 + alpha_bias) + normrnd(0,R_system(1,1));
    y = [measurement;previous_measurement; previous_previous_measurement];
    y_hat = get_output_array(x_hat_k(:,i-1), previous_u,scan_parameters);
    y_hat_series(i) = y_hat(1); 
    y_series(i) = measurement;
    previous_measurement = measurement;
    previous_previous_measurement = previous_measurement;

    
        
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
    
    if(difference + previous_difference < 2)
        G = 0.2;
        G2 = 0.2;
    else
        G = 0.1;
        G2 = 0;
    end
    
    %G = 0.0;
    previous_difference = difference;
    
    normal_u2 = -G*x_hat_k(2,i);
    normal_u3 = -G*x_hat_k(3,i);
    
    u2 = [normal_u2; u2_previous];
    u3 = [normal_u2; u3_previous];

    u2_previous = normal_u2;
    u3_previous = normal_u3;
    
    psi(i) = psi(i-1) + normal_u2;
    theta(i) = theta(i-1) + normal_u3; 
    
    x_hat_k(:,i) = x_hat_k(:,i)+ [0; normal_u2;normal_u3];
    
    
    %Computations related to plotting and motor commands
    alpha_u = alpha_bias;
    beta_u = beta_bias;
 
    psi_offset = atan2d(cosd(alpha_u)*sind(beta_u), cosd(alpha_u)*cosd(beta_u)*cosd(theta(i)) - sind(alpha_u)*cosd(theta(i)));
    theta_offset = asind(cosd(alpha_u)*cosd(beta_u)*sind(theta(i)) + sind(alpha_u)*cosd(theta(i)))-theta(i);
    y = angle_transform(alpha_u, beta_u, theta(i));
    psi_offset = y(1);
    theta_offset = y(2);
    
    scan_psi(i) = psi(i) + psi_offset;
    scan_theta(i) = theta(i) + theta_offset;
    
    Motor_command_psi = scan_psi(i) - scan_psi(i-1);
    Motor_command_theta = scan_theta(i) - scan_theta(i-1);
    
    %Estimated source position
    azimuth = x_hat_k(2,i);
    elevation = x_hat_k(3,i);
    [xe,ye,ze] = sph2cart(azimuth,elevation,r_source);
    
    %Actual source positions
    azimuth = x(2,i);
    elevation = x(3,i);
    [xa,ya,za] = sph2cart(azimuth,elevation,r_source);
    
    %Actual Scan positions 
    xp(i) = cosd(scan_theta(i))*cosd(scan_psi(i));
    yp(i) = cosd(scan_theta(i))*sind(scan_psi(i));
    zp(i) = sind(scan_theta(i));
    plot3(xp,yp,zp,'ro','MarkerFaceColor','y');
    plot3([0 xa],[0 ya],[0 za],'-ro','MarkerFaceColor','r','LineWidth',2);
    plot3([0 xe],[0 ye],[0 ze],'--go','MarkerFaceColor','g','LineWidth',1);
    
    %drawnow;
    frame(i) = getframe;
    t(i) = toc;
    hold off;
end




T = 0.08;
time = T:T:num_iteration*T;



figure;

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
ylim([-20,20]);
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

ylim([-20,20]);
xlabel('Time') % x-axis label
ylabel('$$\hat{y}$$','Interpreter','Latex');
ylim([0,5]);

%figure;
dummy_x = -50:0.5:50;
dummy_y = gaussian_value(dummy_x);
%plot(dummy_x,dummy_y);



%Movie generation
myVideo = VideoWriter('Videos/basic_scan.avi');
open(myVideo);
writeVideo(myVideo, frame);
close(myVideo);