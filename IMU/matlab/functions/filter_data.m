function [ imu_data_filtered ] = filter_data( imu_data_raw )
imu_data = imu_data_raw(:,4:5);
average = mean(imu_data, 1);
derivation = imu_data - repmat(average,size(imu_data,1),1);
derivation = abs(derivation);
max_deri = max(derivation);
flag = derivation < repmat(max_deri,size(derivation,1),1) * 0.8;
flag = all(flag, 2);
imu_data_filtered = imu_data_raw(flag, :);
end

