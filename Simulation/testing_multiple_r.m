%Uses a simpler model g(x,y) = g(x)*g(y) in the simulation 

clear all;
close all;
clc;
%rng(1);
%Scanning parameters
angle_bias(1) = 0;
bias = angle_bias(1);
r_source = 1;


%Initial position parameters
theta(1) = 45;
psi(1) = 45;
my_pos(:,1) = [0;psi(1);theta(1)]; %first entry is considered zero so as to manage the intensity term




%Noise Covariance Matrices and Kalman filter parameters
Q_system = 0.1*[1,0,0;0,10,0;0,0,10];
Q = 0.1*[1,0,0;0,10,0;0,0,10;];
R = eye(3);
R_inv = inv(R);
R_system = 0.1*eye(2);

P(:,:,1) = [1,0,0;0,10,0;0,0,10];
P_current = P(:,:,1);

%System Matrices
A = eye(3);
B = [0,0;1,0;0,1];



scan_radii = [1,2,5,10,15,20,25,30];
%hFig = figure;
%set(hFig, 'Position', [680 678 1400 1050])
%surf(X,Y,real(Z));
num_iteration = 300;
stability_limit = 100;
hold on;
noise_process = [normrnd(zeros(1,num_iteration),Q_system(1,1));normrnd(zeros(1,num_iteration),Q_system(2,2));normrnd(zeros(1,num_iteration),Q_system(3,3));];
noise_measurement = normrnd(zeros(num_iteration,1),R_system(1,1));
scan_phii = [10,20,30,40,50,60,70,80,90];
for ip=1:length(scan_phii)
    phi = scan_phii(ip);
    figure;
    for ir=1:length(scan_radii)
        scan_radius = scan_radii(ir)
        %Previous values needed for initialisation

        scan_theta(1) = theta(1) + scan_radius*sind(bias);
        scan_psi(1) = psi(1) + scan_radius*cosd(bias);
        i=1;
        xp(i) = cosd(scan_theta(i))*cosd(scan_psi(i));
        yp(i) = cosd(scan_theta(i))*sind(scan_psi(i));
        zp(i) = sind(scan_theta(i));

        actual_position(:,1) = [-4,40,50]'; % -4 so as to keep x1 positive 

        x(:,1) = my_pos(:,1)-actual_position(:,1);
        x_hat_k(:,1) = [3,0,0]';

        y_hat_series(1) = 0;

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

        dummy_p = 0;
    
        for i=2:num_iteration
        i
        tic;
        actual_position(:,i) = actual_position(:,i-1)+ 0.08*[0; 1;1] + noise_process(:,i);
        my_pos(:,i) = my_pos(:,i-1)  + [0;u2_k;u3_k];
        x1 = my_pos(1,i)-actual_position(1,i);
        x2_temp = my_pos(2,i)-actual_position(2,i);
        [x3,x2] = angle_transform(my_pos(3,i), x2_temp, -actual_position(3,i));
        %theta(i) = theta(i-1) + u3_k;

        x1_hat_k = x_hat_k(1,i-1);
        x2_hat_k = x_hat_k(2,i-1);
        x3_hat_k = x_hat_k(3,i-1);

        x(:,i) = [x1;x2;x3];
        angle_bias(i) = angle_bias(i-1) + phi;
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
        measurement = exact_measurement_model(x1,x2 + beta_bias,x3 + alpha_bias) + noise_measurement(i);
        y = [measurement;previous_measurement; previous_previous_measurement];
        y_hat = get_output_array(x_hat_k(:,i-1), previous_u,scan_parameters);
        y_hat_series(i) = y_hat(1); 
        y_series(i) = measurement;
        previous_previous_measurement = previous_measurement;
        previous_measurement = measurement;


        % Filtering    
        K = P_current*C'*inv(C*P_current*C' + R);
        K_norm(ir,i) = norm(K);
        x_hat_k(:,i) = x_hat_k(:,i-1)+K*(y-y_hat);
        P(:,:,i) = (eye(3) - K*C)*P_current;
        P_current = P(:,:,i);
        P_norm(ir,i) = norm(P_current);

        difference = abs(y(1)-y_hat(1));
        diff_sum = diff_sum + difference;

        if x_hat_k(1,i) < 0
            x_hat_k(1,i) = 0;
        end

        if(difference + previous_difference < 1)
            G = 1
            G2 = 0.2;
            %scan_radius = 4
        else
            G = 0.2
            G2 = 0;
        end
        %G = 0;
        %G = 0.0;
        previous_difference = difference;

        normal_u2 = -G*x_hat_k(2,i);
        normal_u3 = -G*x_hat_k(3,i);

        u2 = [normal_u2; u2_previous];
        u3 = [normal_u3; u3_previous];
        x_hat_k(:,i) = x_hat_k(:,i)+ [0; normal_u2;normal_u3];

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

        %My position
        azimuth = my_pos(2,i);
        elevation = my_pos(3,i);
        [xe,ye,ze] = sph2cart(azimuth*pi/180,elevation*pi/180,r_source);

        %Actual source positions
        azimuth = actual_position(2,i);
        elevation = actual_position(3,i);
        [xa,ya,za] = sph2cart(azimuth*pi/180,elevation*pi/180,r_source);


        xp(i) = cosd(scan_theta(i))*cosd(scan_psi(i));
        yp(i) = cosd(scan_theta(i))*sind(scan_psi(i));
        zp(i) = sind(scan_theta(i));
    %     if i > 3
    %         h1 = plot3([xp(i-1) xp(i)],[yp(i-1) yp(i)], [zp(i-1) zp(i)],'-bo','MarkerFaceColor','b');
    %     end
    %     h2 = plot3([0 xa],[0 ya],[0 za],'-ro','MarkerFaceColor','r','LineWidth',2);
    %     h3 = plot3([0 xe],[0 ye],[0 ze],'--go','MarkerFaceColor','g','LineWidth',1);
        %drawnow;

        t(i) = toc;
        end
        phi_mesh(ir,ip) = phi;
        r_mesh(ir,ip) = scan_radius;
        p_norm_mesh_mean(ir,ip) = mean(P_norm(ir,stability_limit+1:num_iteration));
        p_norm_mesh_std(ir,ip) = std(P_norm(ir,stability_limit+1:num_iteration));
        plot(P_norm(ir,:));
        legendInfo{ir} = ['r = ',num2str(scan_radius)]; % or whatever is appropriate(['r = ',num2str(scan_radius)])
        hold on
    end
    legend(legendInfo)
    title(['\phi = ',num2str(phi)])
end
%legend([h2,h3,h1],'Transmitter','Center of Scan','Scan points')


T = 0.08;
time = T:T:num_iteration*T;

figure;
[X,Y] = meshgrid(scan_phii,scan_radii);
surf(X,Y,p_norm_mesh_mean);
xlabel('\phi (degree)');
ylabel('r (degree)');
zlabel('||P||_2');


