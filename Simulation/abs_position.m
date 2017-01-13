%Uses a simpler model g(x,y) = g(x)*g(y) in the simulation 

clear all;
close all;
clc;

%Phi is Azimuthal and denoted by x2
%Theta is elevation and denoted by x3
%Position and state varaibles 
Xt(:,1) = [3,230,40]; %Transmitter angular position
Xr(:,1) = [0,245,45]; %Mean receiver initial angular position

%Scanning parameters
angle_bias(1) = 0;
bias = angle_bias(1);
phi = 20;
scan_radius = 10;
alpha_bias = scan_radius*sind(bias);
beta_bias = scan_radius*cosd(bias);
temp_y = angle_transform(alpha_bias,beta_bias,Xr(3,1))

%Initial position parameters (True absolute pointing direction of receiver, actual motor angular positions)
scan_psi(1) = Xr(2,1) + temp_y(1);
scan_theta(1) = Xr(3,1) + temp_y(2);

x_hat_k(:,1) = [2,0,0]';
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




%Previous values needed for initialisation
u2_previous = 0.1;
u3_previous = 0.1;

%local control inputs
u2_k = 0.0;
u3_k = 0.0;

%Global control inputs (without scanning term)
Xr_u2 = 0;
Xr_u3 = 0;

u2 = [u2_k; u2_previous];
u3 = [u3_k; u3_previous];

previous_measurement = 0;
previous_previous_measurement = 0;

diff_sum = 0;
previous_difference = 0;
u2_k = 0;
u3_k = 0;
Motor_command_psi = 0;
Motor_command_theta = 0;



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
    
    %Calculation of states(x) from the absolute states
    Xr(:,i) = Xr(:,i-1) + [0; Xr_u2;Xr_u3];
	Xt(:,i) = Xt(:,i-1)+  + 0.0*[0; 1;1] + 0*[normrnd(0,Q_system(1,1)); normrnd(0,Q_system(2,2));normrnd(0,Q_system(3,3))];
    
    beta_x = Xt(2,i)-Xr(2,i);
    alpha_x = Xt(3,i);
    theta_x = -Xr(3,i);
    temp_y = angle_transform(alpha_x,beta_x,theta_x);
    
    x(1,i) = Xt(1,i)-Xr(1,i); 
    x(2,i) = temp_y(1); 
    x(3,i) = temp_y(2); 
    x1 = x(1,i);
    x2 = x(2,i);
    x3 = x(3,i);
    
	x1_hat_k = x_hat_k(1,i-1);
    x2_hat_k = x_hat_k(2,i-1);
    x3_hat_k = x_hat_k(3,i-1);
    
    angle_bias(i) = angle_bias(i-1) + phi;
    bias = angle_bias(i);
	previous_alpha_bias = scan_radius*sind(bias-phi);
	previous_beta_bias = scan_radius*cosd(bias-phi);
	alpha_bias = scan_radius*sind(bias);
	beta_bias = scan_radius*cosd(bias);

    %Calculation of motor commands
    %Motor should move by the control command plus the effect of scanning: u + bias - previous_bias
    alpha_u = u3_k + alpha_bias - previous_alpha_bias;
    beta_u = u2_k + beta_bias - previous_beta_bias;
    temp_y = angle_transform(alpha_u,beta_u ,scan_theta(i-1));
    Motor_command_psi = temp_y(1);
    Motor_command_theta = temp_y(2);
    scan_psi(i) = scan_psi(i-1) + Motor_command_psi;
    scan_theta(i) = scan_theta(i-1) + Motor_command_theta;
     
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
    
    G = 0.0;
    previous_difference = difference;
    
    u2_k = -G*x_hat_k(2,i);
    u3_k = -G*x_hat_k(3,i);
    
    u2 = [u2_k; u2_previous];
    u3 = [u2_k; u3_previous];

    u2_previous = u2_k;
    u3_previous = u3_k;
    
    x_hat_k(:,i) = x_hat_k(:,i)+ [0; u2_k;u3_k];
    
    %Incorporating these control inputs into angular positions
    temp_y = angle_transform(u3_k,u2_k,Xr(3,i));
    %Absolute mean pointing position changes according to following 
    Xr_u2 = temp_y(1);
    Xr_u3 = temp_y(2);
    
    %Positions computation for plotting 
    %Estimated source position
    alpha_x_hat = x_hat_k(3,i);
    beta_x_hat = x_hat_k(2,i);
    temp_y = angle_transform(alpha_x_hat,beta_x_hat,Xr(3,i));
    azimuth = Xr(2,i) + temp_y(1);
    elevation = Xr(3,i) + temp_y(2);
    [xe,ye,ze] = sph2cart(azimuth,elevation,r_source);
    
    %Actual source positions
    azimuth = Xt(2,i);
    elevation = Xt(3,i);
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