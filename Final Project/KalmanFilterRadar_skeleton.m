clear;
functions = utility_functions;
[allData, ~, ~] = Sonnenstrasse_sim();
og_trajectory = functions.get_trajectory(allData, 2);
og_measures = functions.get_aggregated_measures(allData, 2);

% Remove rows with NaN values from og_measures
og_measures = og_measures(~any(isnan(og_measures), 2), :);

% Plotting
subplot(3,1,1);
plot(og_trajectory(:, 1), og_trajectory(:, 2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
title('Trajectory');
xlabel('X (m)');
ylabel('Y (m)');

subplot(3,1,2);
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
Q = eye(4); % Process noise covariance matrix
R = eye(2); % Measurement noise covariance matrix

% Initialize state estimate and covariance matrix
x_est = [trajectory(1, 1); trajectory(1, 2); 0; 0]; % Initial state estimate
P = eye(4); % Initial covariance matrix

% Initialize variables to store estimated trajectory
estimated_trajectory = zeros(size(measures, 1), 2);

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
    estimated_trajectory(i, :) = [x_est(1), x_est(2)];
end

% Plotting
subplot(3,1,3);
plot(estimated_trajectory(:, 1), estimated_trajectory(:, 2), 'bo-', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
title('Estimated Trajectory');
xlabel('X (m)');
ylabel('Y (m)');

