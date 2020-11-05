%% Initalization
close all
clear all

% Read in our toolboxes
addpath('functions')

titlestr = 'GY85';
txt_file = 'imu_data.txt';
imu_data = importdata(txt_file);
imu_data = imu_data(:,2:7);
%imu_data = filter_data(imu_data);

gry_data = imu_data(:,4:6);

maxNumM = 100;
L = size(gry_data, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

t0 = 0.03366474699760932;

[tau_x, adev_x] = allan_variance(gry_data(:,1), m, t0);
[tau_y, adev_y] = allan_variance(gry_data(:,2), m, t0);
[tau_z, adev_z] = allan_variance(gry_data(:,3), m, t0);

%% Get the calculated sigmas


fprintf('=> plotting gyroscope.\n')
% [fh2,sigma_g,sigma_ga] = gen_chart_average(tau_x,adev_x,...
%                                     adev_y,adev_z,...
%                                     titlestr,'gyroscope','rad/s',...
%                                     'rad/s^1sqrt(Hz)','rad/s^2sqrt(Hz)');

[fh2,sigma_g,sigma_ga] = gen_chart(tau_y,adev_y,...
                                    titlestr,'gyroscope','rad/s',...
                                    'rad/s^1sqrt(Hz)','rad/s^2sqrt(Hz)');

                                

%% Print out for easy copying
fprintf('=> final results\n');
% Gryoscope
fprintf('gyroscope_noise_density     = %.8f\n',sigma_g);
fprintf('gyroscope_random_walk       = %.8f\n',sigma_ga);








