clear all;
close all;
clc;

load 'data/last.mat'
size_input = size(X);
length = size_input(1);


subplot(2,1,1);
surf(X(1:end-1,:),Y(1:end-1,:),p_norm_mesh_mean_all(1:end-1,:));
xlabel('\delta_{\psi} (degree)');
ylabel('\delta_r (degree)');
zlabel('||P||_2');
line_styles = ['-','--',':','-.','--','-'];
line_widths = [1,2,1,2,1,2];
subplot(2,1,2);
for i=1:length
    scan_radius = scan_radii(i)
    [x,y] = find([2,5,10,15,25,35]==scan_radius)
    if ~isempty(y)
         if scan_radius == 35
            linewidth = 2;
            legendInfo{y} = ['adaptive \delta_r'];   
        else
            linewidth = 1;
            str = num2str(scan_radius);
            legendInfo{y} = ['\delta_r = ',str];   
         end
        linestyle = line_styles(y);
        linewidth = line_widths(y);
        plot(X(1,:),p_norm_mesh_mean_all(i,:),linestyle,'LineWidth',linewidth);
        hold on
       
    end
end
 xlabel('\delta_{\psi} (degree)');
 ylabel('||P||_2');
legend(legendInfo)

figure;
subplot(2,1,1);
surf(X(1:end-1,:),Y(1:end-1,:),control_effort_all(1:end-1,:));
xlabel('\delta_{\psi} (degree)');
ylabel('\delta_r (degree)');
zlabel('U_n');

subplot(2,1,2);
for i=1:length
   scan_radius = scan_radii(i)
    [x,y] = find([2,5,10,15,25,35]==scan_radius)
    if ~isempty(y)
         if scan_radius == 35
            linewidth = 2;
            legendInfo{y} = ['adaptive \delta_r'];   
        else
            linewidth = 1;
            str = num2str(scan_radius);
            legendInfo{y} = ['\delta_r = ',str];   
         end
        linestyle = line_styles(y);
        linewidth = line_widths(y);
        plot(X(1,:),control_effort_all(i,:),linestyle,'LineWidth',linewidth);
        hold on
    end
end
legend(legendInfo)
xlabel('\delta_{\psi} (degree)');
ylabel('U_n');

figure;
subplot(2,1,1);
surf(X(1:end-1,:),Y(1:end-1,:),difference_all(1:end-1,:));
xlabel('\delta_{\psi} (degree)');
ylabel('\delta_r (degree)');
zlabel('V_n');

subplot(2,1,2);
for i=1:length
    scan_radius = scan_radii(i)
    [x,y] = find([2,5,10,15,25,35]==scan_radius)
    if ~isempty(y)
         if scan_radius == 35
            linewidth = 2;
            legendInfo{y} = ['adaptive \delta_r'];   
        else
            linewidth = 1;
            str = num2str(scan_radius);
            legendInfo{y} = ['\delta_r = ',str];   
         end
        linestyle = line_styles(y);
        linewidth = line_widths(y);
        plot(X(1,:),difference_all(i,:),linestyle,'LineWidth',linewidth);
        hold on
    end
end
legend(legendInfo)
xlabel('\delta_{\psi} (degree)');
ylabel('V_n');



