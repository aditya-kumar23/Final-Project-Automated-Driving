clear;
functions = utility_functions;

[allData, ~, ~] = Sonnenstrasse_sim();

og_trajectory = functions.get_trajectory(allData, 2);
og_measures = functions.get_aggregated_measures(allData, 1);
og_velocity = functions.compute_actual_velocity(allData,2,1);

% Remove rows with NaN values from og_measures
og_measures = og_measures(~any(isnan(og_measures), 2), :);
og_velocity = og_velocity(~any(isnan(og_velocity), 2), :);

%disp(og_velocity);

% Plotting
subplot(4,1,1);
plot(og_trajectory(:, 1), og_trajectory(:, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
title('Trajectory');
xlabel('X (m)');
ylabel('Y (m)');

subplot(4,1,2);
plot(og_measures(:, 1), og_measures(:, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Measurements');
xlabel('X (m)');
ylabel('Y (m)');

trajectory = [og_trajectory(:, 1), og_trajectory(:, 2)];
measures = [og_measures(:,1), og_measures(:,2)];

% Define system matrices
dt = 1; % Time step
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % State transition matrix
H = [1 0 0 0; 0 1 0 0]; % Measurement matrix
Q = diag([0.1, 0.1, 1, 1]); % we consider that velocity is more noisy %Q = eye(4); % Process noise covariance matrix
R = eye(2); % Measurement noise covariance matrix

% Initialize state estimate and covariance matrix
x_est = [trajectory(1, 1); trajectory(1, 2); 0; 0]; % Initial state estimate
P = eye(4); % Initial covariance matrix

% Initialize variables to store estimated trajectory
estimated_trajectory = zeros(size(measures, 1), 2);
estimated_velocity = zeros(size(measures, 1), 2);

% Kalman filter loop
for i = 1:size(measures, 1)
    % Prediction update
    x_pred = A * x_est;
    P_pred = A * P * A' + Q;

    % Measurement update
    K = P_pred * H' * inv(H * P_pred * H' + R);
    z = measures(i, :)'; % Measurement vector
    x_est = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K * H) * P_pred;

    % Store estimated trajectory
    estimated_trajectory(i, :) = [x_est(1), x_est(2)]; %position
    estimated_velocity(i, :) = [x_est(3), x_est(4)];   % Velocity
end
threshold = 2; % Number of standard deviations for outlier detection
estimated_velocity = detect_outliers_mad(estimated_velocity, threshold);

% Plotting position
subplot(4,1,3);
plot(estimated_trajectory(:, 1), estimated_trajectory(:, 2), 'bo-', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
title('Estimated Trajectory');
xlabel('X (m)');
ylabel('Y (m)');

% Plotting velocity
subplot(4,1,4);
plot(1:size(estimated_velocity, 1), estimated_velocity(:, 1), 'r', 'DisplayName', 'X Velocity');
hold on;
plot(1:size(estimated_velocity, 1), estimated_velocity(:, 2), 'b', 'DisplayName', 'Y Velocity');
legend;
xlabel('Time Step');
ylabel('Velocity (m/s)');
title('Estimated Velocity Over Time');


% % Find the minimum number of iterations based on the size of available data
% num_iterations = min(size(og_velocity, 1), size(estimated_velocity, 1));
% 
% % Initialize error storage with correct size
% velocity_error = zeros(num_iterations, 2);
% 
% % Loop through and compute velocity error
% for i = 1:num_iterations
%     % Extract actual velocity for this time step
%     actual_vx = og_velocity(i, 1);
%     actual_vy = og_velocity(i, 2);
% 
%     % Extract estimated velocity from Kalman filter
%     estimated_vx = estimated_velocity(i, 1);  % Correct index here for x_velocity
%     estimated_vy = estimated_velocity(i, 2);  % Correct index here for y_velocity
% 
%     % Compute error
%     velocity_error(i, 1) = abs(actual_vx - estimated_vx);
%     velocity_error(i, 2) = abs(actual_vy - estimated_vy);
% end


% Plot actual vs estimated velocity for x-direction
figure;
subplot(2, 1, 1);
plot(1:size(og_velocity, 1), og_velocity(:, 1), 'b', 'DisplayName', 'Actual v_x');
hold on;
plot(1:size(estimated_velocity, 1), estimated_velocity(:, 1), 'r', 'DisplayName', 'Estimated v_x');
xlabel('Time step');
ylabel('Velocity (m/s)');
legend;
title('Comparison of x-velocity (actual vs estimated)');

% Plot actual vs estimated velocity for y-direction
subplot(2, 1, 2);
plot(1:size(og_velocity, 1), og_velocity(:, 2), 'b', 'DisplayName', 'Actual v_y');
hold on;
plot(1:size(estimated_velocity, 1), estimated_velocity(:, 2), 'r', 'DisplayName', 'Estimated v_y');
xlabel('Time step');
ylabel('Velocity (m/s)');
legend;
title('Comparison of y-velocity (actual vs estimated)');

% function filtered_measures = detect_outliers_zscore(measures, threshold)
%     % Calculate the mean and standard deviation
%     mean_meas = mean(measures);
%     std_meas = std(measures);
% 
%     % Initialize filtered measures
%     filtered_measures = measures;
% 
%     % Loop through each measurement
%     for i = 1:size(measures, 1)
%         % Calculate Z-score for x and y measurements
%         z_score_x = (measures(i, 1) - mean_meas(1)) / std_meas(1);
%         z_score_y = (measures(i, 2) - mean_meas(2)) / std_meas(2);
% 
%         % Check if Z-score exceeds the threshold
%         if abs(z_score_x) > threshold
%             filtered_measures(i, 1) = mean_meas(1); % Replace with mean
%         end
%         if abs(z_score_y) > threshold
%             filtered_measures(i, 2) = mean_meas(2); % Replace with mean
%         end
%     end
% end
function filtered_measures = detect_outliers_mad(measures, threshold)
    % Calculate the median and MAD
    median_meas = median(measures);
    mad_meas = mad(measures, 1); % MAD with scaling factor of 1

    % Initialize filtered measures
    filtered_measures = measures;

    % Loop through each measurement
    for i = 1:size(measures, 1)
        % Check if the measurement is an outlier
        if abs(measures(i, 1) - median_meas(1)) > threshold * mad_meas(1)
            filtered_measures(i, 1) = median_meas(1); % Replace with median
        end
        if abs(measures(i, 2) - median_meas(2)) > threshold * mad_meas(2)
            filtered_measures(i, 2) = median_meas(2); % Replace with median
        end
    end
end



