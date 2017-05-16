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

size_input = size(X);
length = size_input(1);

figure;
for i=1:length
    scan_radius = scan_radii(i)
    if scan_radius == 35
        linewidth = 2;
    else
        linewidth = 1;
    end
    plot(p_norm_mesh_mean_all(i,:),'LineWidth',linewidth);
    hold on
    legendInfo{i} = ['r = ',num2str(scan_radius)];   
end
legend(legendInfo)
figure;
for i=1:length
    scan_radius = scan_radii(i)
    if scan_radius == 35
        linewidth = 2;
    else
        linewidth = 1;
    end
    plot(control_effort_all(i,:),'LineWidth',linewidth);
    
    hold on
    legendInfo{i} = ['r = ',num2str(scan_radius)];   
end
legend(legendInfo)
