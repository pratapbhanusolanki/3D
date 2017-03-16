clear all;
close all;
clc;

unzip('model_data_step3.npz','model_data')
model_data = readNPY('model_data/model_data.npy');
figure
X = readNPY('model_data/psi_data.npy');
Y = readNPY('model_data/theta_data.npy');
surf(X,Y,model_data)

size_data = size(model_data);

for i=1:size_data(1)
    [M,ind] = max(model_data(i,:));
    shift_model_data(i,:) = circshift(model_data(i,:),ceil(size_data(2)/2)-ind,2);
    i
    ind
end
figure
surf(X,Y,shift_model_data);
figure
plot(shift_model_data(2,:));
figure
plot(shift_model_data(:,201));

