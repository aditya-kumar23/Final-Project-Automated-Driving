function [estimated_trajectory, estimated_velocity] = estimate_trajectory(allData, actor_id, ego_id, timeArray)
    
functions = utility_functions;

    % Get the original trajectory and measurements
    og_trajectory = functions.get_trajectory(allData, actor_id);
    og_measures = functions.get_aggregated_measures(allData, ego_id);
    
    % Remove rows with NaN values from og_measures
    og_measures = og_measures(~any(isnan(og_measures), 2), :);

   
    % Define system matrices
    % dt = 1; % Time step
    % A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % State transition matrix
    H = [1 0 0 0; 0 1 0 0]; % Measurement matrix
    Q = diag([0.1, 0.1, 0, 0]); % Process noise covariance matrix
    R = eye(2); % Measurement noise covariance matrix
    
    % Initialize state estimate and covariance matrix
    x_est = [og_trajectory(1, 1); og_trajectory(1, 2); 0; 0]; % Initial state estimate
    P = eye(4); % Initial covariance matrix

    % Initialize variables to store estimated trajectory
    estimated_trajectory = zeros(size(og_measures, 1), 2);
    estimated_velocity = zeros(size(og_measures, 1), 2);

    % Kalman filter loop
    for i = 1:size(og_measures, 1)
        if i == 1
            dt = 0.0400;  % Use a default value for the first iteration
        else
            dt = max(timeArray(i) - timeArray(i-1), 0.001);  % Ensure dt is never zero
        end
        % Update the state transition matrix (A) based on the current dt
        A = [1 0 dt 0; 0 1 0 dt; 0 0 1  0; 0 0 0  1];
        
        % Prediction update
        x_pred = A * x_est;
        P_pred = A * P * A' + Q;

        % Measurement update
        K = P_pred * H' * inv(H * P_pred * H' + R);
        z = og_measures(i, :)'; % Measurement vector
        x_est = x_pred + K * (z - H * x_pred);
        P = (eye(4) - K * H) * P_pred;

        % Store estimated trajectory
        estimated_trajectory(i, :) = [x_est(1), x_est(2)]; % Position
        estimated_velocity(i, :) = [x_est(3), x_est(4)];   % Velocity
    end

    % Optional: Outlier detection can be included here if needed
    threshold = 2; % Number of standard deviations for outlier detection
    %estimated_velocity = detect_outliers_mad(estimated_velocity, threshold);
end

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
