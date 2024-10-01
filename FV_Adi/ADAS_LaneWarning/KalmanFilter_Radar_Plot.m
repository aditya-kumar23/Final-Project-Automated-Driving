clear;
functions = utility_functions;
threshold = 2; % Number of standard deviations for outlier detection

[allData, ~, ~] = simulationEnvironment_FV_N();

% Extract the time steps from allData1,1
timeArray = arrayfun(@(x) x.Time, allData);

actor_id = 1;
ego_id = 2; 

og_trajectory = functions.get_trajectory(allData,1);
og_measures = functions.get_aggregated_measures(allData,ego_id);
og_velocity = functions.compute_actual_velocity(allData, actor_id, timeArray);

% Remove rows with NaN values from og_measures
og_measures = og_measures(~any(isnan(og_measures), 2), :);
og_velocity = og_velocity(~any(isnan(og_velocity), 2), :);

%disp(og_velocity);

figure; % Create a new figure

% Plotting
subplot(4,1,1);
plot(og_trajectory(:, 1), og_trajectory(:, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
title('Trajectory');
xlabel('X (m)');
ylabel('Y (m)');
grid on; 

subplot(4,1,2);
plot(og_measures(:, 1), og_measures(:, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
title('Measurements');
xlabel('X (m)');
ylabel('Y (m)');
grid on;

trajectory = [og_trajectory(:, 1), og_trajectory(:, 2)];

measures = [og_measures(:,1), og_measures(:,2)];

velocity = [og_velocity(:, 1), og_velocity(:, 2)];

%measures = detect_outliers_mad(measures, threshold);

% Define system matrices

H = eye(4); % Measurement matrix now includes position and velocity
Q = diag([1, 1, 1, 1]); % Process noise covariance matrix
R = diag([0.5, 0.5, 0.1, 0.1]); % Measurement noise covariance matrix

% Initialize state estimate and covariance matrix
x_est = [trajectory(1, 1); trajectory(1, 2); velocity(1,1); velocity(1,2)]; % Initial state estimate (position and velocity)
P = eye(4); % Initial covariance matrix

% Initialize variables to store estimated trajectory and velocity
estimated_trajectory = zeros(size(measures, 1), 2);
estimated_velocity = zeros(size(measures, 1), 2);

% Kalman filter loop
 for i = 1:size(og_measures, 1)
        if i == 1
            dt = 0.0400;  % Use a default value for the first iteration (you can also choose a small value)
        else
            dt = max(timeArray(i) - timeArray(i-1), 0.001);  % Ensure dt is never zero
        end

    % Update the state transition matrix (A) based on the current dt
    A = [1 0 dt 0; 
         0 1 0 dt; 
         0 0 1  0; 
         0 0 0  1];

    % Prediction update
    x_pred = A * x_est;
    P_pred = A * P * A' + Q;

    % Measurement update
    K = P_pred * H' / (H * P_pred * H' + R);
    z = [measures(i, :)'; velocity(i, :)']; % Measurement vector
    x_est = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K * H) * P_pred;

    % Store estimated position and velocity
    estimated_trajectory(i, :) = [x_est(1), x_est(2)]; % Position
    estimated_velocity(i, :) = [x_est(3), x_est(4)];   % Velocity
end


%estimated_velocity = detect_outliers_mad(estimated_velocity, threshold);

% Plotting estimated trajectory
subplot(4, 1, 3);
plot(estimated_trajectory(:, 1), estimated_trajectory(:, 2), 'bo-', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
title('Estimated Trajectory');
xlabel('X (m)');
ylabel('Y (m)');
grid on;

% Plotting estimated velocity
subplot(4, 1, 4);
plot(1:size(estimated_velocity, 1), estimated_velocity(:, 1), 'r', 'DisplayName', 'X Velocity');
hold on;
plot(1:size(estimated_velocity, 1), estimated_velocity(:, 2), 'b', 'DisplayName', 'Y Velocity');
legend;
xlabel('Time Step');
ylabel('Velocity (m/s)');
title('Estimated Velocity Over Time');
grid on;

% Compare actual vs estimated velocity
figure; % Create another figure for comparisons
subplot(2, 1, 1);
plot(timeArray(1:size(og_velocity, 1)), og_velocity(:, 1), 'b', 'DisplayName', 'Actual v_x');
hold on;
plot(timeArray(1:size(estimated_velocity, 1)), estimated_velocity(:, 1), 'r', 'DisplayName', 'Estimated v_x');
xlabel('Time step (s)');
ylabel('Velocity (m/s)');
legend;
title('Comparison of x-velocity (actual vs estimated)');
grid on;

subplot(2, 1, 2);
plot(timeArray(1:size(og_velocity, 1)), og_velocity(:, 2), 'b', 'DisplayName', 'Actual v_y');
hold on;
plot(timeArray(1:size(estimated_velocity, 1)), estimated_velocity(:, 2), 'r', 'DisplayName', 'Estimated v_y');
xlabel('Time step (s)');
ylabel('Velocity (m/s)');
legend;
title('Comparison of y-velocity (actual vs estimated)');
grid on;

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

% function filtered_measures = detect_outliers_mad(measures, threshold)
%     % Calculate the median and MAD
%     median_meas = median(measures);
%     mad_meas = mad(measures, 1); % MAD with scaling factor of 1
% 
%     % Initialize filtered measures
%     filtered_measures = measures;
% 
%     % Loop through each measurement
%     for i = 1:size(measures, 1)
%         % Check if the measurement is an outlier
%         if abs(measures(i, 1) - median_meas(1)) > threshold * mad_meas(1)
%             filtered_measures(i, 1) = median_meas(1); % Replace with median
%         end
%         if abs(measures(i, 2) - median_meas(2)) > threshold * mad_meas(2)
%             filtered_measures(i, 2) = median_meas(2); % Replace with median
%         end
%     end
% end



