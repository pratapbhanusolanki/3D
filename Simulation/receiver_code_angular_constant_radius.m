%Uses a simpler model g(x,y) = g(x)*g(y) in the simulation 

clear all;
close all;
clc;

%Scanning parameters
angle_bias(1) = 0;
bias = angle_bias(1);
phi = 30;
scan_radius = 20;%10;
r_source = 1;


%Initial position parameters
theta(1) = 35;
psi(1) = 40;
my_pos(:,1) = [0;psi(1);theta(1)]; %first entry is considered zero so as to manage the intensity term

scan_theta(1) = theta(1) + scan_radius*sind(bias);
scan_psi(1) = psi(1) + scan_radius*cosd(bias);
i=1;
xp(i) = cosd(scan_theta(i))*cosd(scan_psi(i));
yp(i) = cosd(scan_theta(i))*sind(scan_psi(i));
zp(i) = sind(scan_theta(i));

actual_position(:,1) = [-5,55,55]'; % -4 so as to keep x1 positive 

x(:,1) = my_pos(:,1)-actual_position(:,1);
x_hat_k(:,1) = [3,0,0]';
x_I_hat_k(:,1) = x_hat_k(:,1);

y_hat_series(1) = 0;

%Noise Covariance Matrices and Kalman filter parameters
Q_system = 0.01*[1,0,0;0,10,0;0,0,10];
Q = 0.1*[1,0,0;0,10,0;0,0,10;];
R = 1;
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

u2_k = 0.0;
u3_k = 0.0;

u2 = [u2_k; u2_previous];
u3 = [u3_k; u3_previous];

previous_measurement = 0;
previous_previous_measurement = 0;

diff_sum = 0.2;
previous_difference = 0;
normal_u2 = 0;
normal_u3 = 0;


num_iteration = 200;
T = 0.08;
time = T:T:num_iteration*T;

[X,Y] = meshgrid(-1:0.01:1);
Z = real(sqrt(1-X.*X - Y.*Y));
hFig = figure;
%set(hFig, 'Position', [680 678 1400 1050])
%surf(X,Y,real(Z));
hold on;
xlim([-inf 30]);
%legend([h3,h2,h1],'Transmitter','Center of Scan','Scan points')
set(gca,'FontSize',12)
xlabel('$$x_1, \hat{x}_2$$ (degree)','Interpreter','Latex') % x-axis label
ylabel('$$x_2, \hat{x}_3$$ (degree)','Interpreter','Latex');
grid on;
pbaspect([1 1 1])
for i=2:num_iteration
    i
	tic;
	actual_position(:,i) = actual_position(:,i-1)+ 0.08*[0; 1;1] + [normrnd(0,Q_system(1,1)); normrnd(0,Q_system(2,2));normrnd(0,Q_system(3,3))];
    my_pos(:,i) = my_pos(:,i-1)  + [0;u2_k;u3_k];
    x1 = my_pos(1,i)-actual_position(1,i);
    x2_temp = my_pos(2,i)-actual_position(2,i);
    [x3,x2] = angle_transform(my_pos(3,i), x2_temp, -actual_position(3,i));
	%theta(i) = theta(i-1) + u3_k;
        
    x(:,i) = [x1;x2;x3];
    angle_bias(i) = angle_bias(i-1) + phi;
    scan_radius = max(2,floor(0.7*scan_radius + 0.3*min(floor(diff_sum*100),10)));
    %scan_radius = 7;
    scan_radii(i) = scan_radius;
    bias = angle_bias(i);
	previous_alpha_bias = scan_radius*sind(bias-phi);
	previous_beta_bias = scan_radius*cosd(bias-phi);
	alpha_bias = scan_radius*sind(bias);
	beta_bias = scan_radius*cosd(bias);
    
    previous_u = [u2,u3];
    scan_parameters = [scan_radius, bias, phi];
    
    C = get_C_matrix(x_hat_k(:,i-1),previous_u,scan_parameters);
    rank_C(i) = rank(C);
    rank(C);
    P_current = A*P_current*A' + Q;
    
    % Output calculation
    measurement = exact_measurement_model(x1,x2 + beta_bias,x3 + alpha_bias) + normrnd(0,R_system(1,1));
    y = measurement;
    y_hat = get_output_array(x_hat_k(:,i-1), previous_u,scan_parameters);
    y_hat_series(i) = y_hat(1); 
    y_series(i) = measurement;
    
  
    % Filtering    
    K = P_current*C'*inv(C*P_current*C' + R);
    x_hat_k(:,i) = x_hat_k(:,i-1)+K*(y-y_hat);
    if x_hat_k(1,i) < 0
        x_hat_k(1,i) = 0;
    end
    x_I_hat_k(:,i) = x_I_hat_k(:,i-1) + T*x_hat_k(:,i-1);
    P(:,:,i) = (eye(3) - K*C)*P_current;
    P_current = P(:,:,i);

    difference(i) = abs((y-y_hat)/y);
    min_ind = max(i-2,1)
    diff_sum = sum(difference(min_ind:i))/3;
    
    if x_hat_k(1,i) < 0
        x_hat_k(1,i) = 0;
    end
    
    if(diff_sum < 0.3)
        G = 0.5
        Gi = 0.1;
    else
        G = 0.0
        Gi = 0;
    end
    
    %G = 0.0;
    
%     G = 0.5
%     Gi = 0.01
    
    previous_difference = difference;
    
    normal_u2 = -G*x_hat_k(2,i)-Gi*x_I_hat_k(2,i);
    normal_u3 = -G*x_hat_k(3,i)-Gi*x_I_hat_k(3,i);
    
    u2 = [normal_u2; u2_previous];
    u3 = [normal_u3; u3_previous];
    x_hat_k(:,i) = x_hat_k(:,i)+ [0; normal_u2;normal_u3];

    u2_previous = normal_u2;
    u3_previous = normal_u3;
    
    [dummy_u3,u2_k] = angle_transform(normal_u3, normal_u2, theta(i-1));
    u3_k = dummy_u3 - theta(i-1);
    
    psi(i) = psi(i-1) + u2_k;
    theta(i) = theta(i-1) + u3_k; 

    xp(i) = x_hat_k(2,i) + beta_bias;
    yp(i) = x_hat_k(3,i) + alpha_bias;
    ux = xp(i)-x(2,i)-xp(i-1)+x(2,i-1);
    vy = yp(i)-x(3,i)-yp(i-1)+x(3,i-1);
    
   if i > 3
        %h1 = plot([xp(i-1) xp(i)]-[x2,x(2,i-1)],[yp(i-1) yp(i)]-[x3,x(3,i-1)],'--bo','MarkerFaceColor','b','LineWidth',1);
        h1 = quiver(xp(i-1)- x(2,i-1),yp(i-1)- x(3,i-1),ux,vy,'-g','LineWidth',1,'MaxHeadSize',2);
        %h1 = quiver(xp(i),yp(i),[xp(i-1)-xp(i)],[yp(i-1)-yp(i)],'-bo','MarkerFaceColor','b','LineWidth',1);
    lgd = legend([h3,h2,h1],'Transmitter','Center of Scan','Scan points');
    lgd.Location = 'southeast';
    frame(i-3) = getframe;
   end
    h2 = plot([x_hat_k(2,i-1) x_hat_k(2,i)]-[x2,x(2,i-1)],[x_hat_k(3,i-1) x_hat_k(3,i)]-[x3,x(3,i-1)],'--bs','LineWidth',1,'MarkerSize',10);
    h3 = plot([0,0],[0,0],'ro','MarkerFaceColor','r', 'MarkerSize',10);
    %title('iteration = '+i);
    drawnow;
    t(i) = toc;
end

legend([h3,h2,h1],'Transmitter','Center of Scan','Scan points')
set(gca,'FontSize',18)
xlabel('$$x_1, \hat{x}_2$$ (degree)','Interpreter','Latex') % x-axis label
ylabel('$$x_2, \hat{x}_3$$ (degree)','Interpreter','Latex');
grid on;




figure;

subplot(4,1,1);
%plot(time,x(1,:));
hold on;
%plot(x_hat_data(1,1:num_iteration),'r');
plot(time,x(1,:),'r');
ylim([0, 6]);
plot(time,x_hat_k(1,:),'--b');
%title('Convergence');
%xlabel('Time') % x-axis label
ylabel('$$x_1, \hat{x}_1$$ (V)','Interpreter','Latex');
legend('Original state','Extended Kalman Filter')
set(gca,'FontSize',18)
xlim([time(1),time(end)]);
ylim([0,8]);

subplot(4,1,2);
hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
plot(time,x(2,:),'r');
plot(time,x_hat_k(2,:),'--b');
%ylim([-20,20]);
%xlabel('Time') % x-axis label
ylabel('$$x_2, \hat{x}_2$$ (degree)','Interpreter','Latex');
set(gca,'FontSize',18)
xlim([time(1),time(end)]);

subplot(4,1,3);

hold on;
%plot(time,x(2,:));
%plot(x_hat_data(2,1:num_iteration),'r');
%plot(-20*control_data(1:num_iteration),'k');
plot(time,x(3,:),'r');
plot(time,x_hat_k(3,:),'--b');
ylim([-20,20]);
%xlabel('Time') % x-axis label
ylabel('$$x_3, \hat{x}_3$$ (degree)','Interpreter','Latex');
set(gca,'FontSize',18)
xlim([time(1),time(end)]);

subplot(4,1,4);
hold on;
%plot(time,x(2,:));
%plot(y_hat_data(1,1:num_iteration),'r');
set(gca,'FontSize',18)

plot(time, y_series(1,1:num_iteration),'r');
%plot(time, measurement_data,'k');
plot(time, y_hat_series,'--b')
legend('Simulated measurement','Estimated measurement')

ylim([-20,20]);
xlabel('Time (s)') % x-axis label
ylabel('$$y, \hat{y}$$ (V)','Interpreter','Latex');
ylim([0,5]);
xlim([time(1),time(end)]);

%figure;
dummy_x = -50:0.5:50;
dummy_y = gaussian_value(dummy_x);
%plot(dummy_x,dummy_y);
%Movie generation
myVideo = VideoWriter('Videos/adaptive_scan.avi');
open(myVideo);
writeVideo(myVideo, frame);
close(myVideo);
