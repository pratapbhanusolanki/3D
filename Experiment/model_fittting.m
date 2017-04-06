clear all;
close all;
clc;

unzip('model_data_step3.npz','model_data')
model_data = readNPY('model_data/model_data.npy');
%figure
X = readNPY('model_data/psi_data.npy');
Y = readNPY('model_data/theta_data.npy');
%X = readNPY('model_data/theta_data.npy');
size_data = size(model_data);

for i=1:size_data(1)
    [M,ind] = max(model_data(i,:));
    shift_model_data(i,:) = circshift(model_data(i,:),ceil(size_data(2)/2)-ind,2);
    i;
    ind;
end
figure
surf(X,Y,model_data);
figure
surf(X,Y,shift_model_data);
% figure
% plot(shift_model_data(2,:));
% figure
% plot(shift_model_data(:,201));
% y = model_data(1,:)/max(model_data(1,:));
% z = circshift(y',-26);

% figure
% x = -75:0.5:75;
% 
% for i=1:length(x)
%     y_old(i) = gaussian_value(x(i));
%     y_new(i) = gaussian_value2(x(i));
% end
% 
% hold on;
% plot(x,y_old,'r');
% plot(x,y_new,'b');
% plot(x,z,'g');

[M_array,ind_array] = max(shift_model_data);
[M,ind] = max(M_array);
My_ind = ind;
Mx_ind = ind_array(ind);
figure;

X_Data = circshift(shift_model_data(Mx_ind,:),Mx_ind-My_ind);
Y_Data = circshift(shift_model_data(:,My_ind),0);
plot(X_Data,'LineWidth',1);
hold on
plot(Y_Data,'LineWidth',1)

plot((X_Data + Y_Data')/2,'r','LineWidth',2);

y_data = (X_Data + Y_Data')/2;
[m,i] = max(y_data);
 x = -i:1:60-i;
 y = y_data/m;
 
figure;
plot(x,y,'g','LineWidth',0.5);

f = fit(x',y','gauss2')
hold on;

plot(f,x,y)





