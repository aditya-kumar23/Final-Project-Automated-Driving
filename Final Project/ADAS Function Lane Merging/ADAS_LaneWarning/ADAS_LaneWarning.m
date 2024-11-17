
%% this is ADAS with kalman filter data
clear;

% Call the simulation environment function to get the data
[allData, ~, ~] = simulationEnvironment_FV_N();

 % Extract the time steps from allData
timeArray = arrayfun(@(x) x.Time, allData);

% Call the function to get the estimated trajectory and velocity
trajectoryIndex = 1; % Index for the truck trajectory
measureIndex = 2;    % Index for measurements
[estimated_trajectory, estimated_velocity] = estimate_trajectory(allData, trajectoryIndex, measureIndex, timeArray);

% Define the actor_id for the truck and the ego vehicle
actor_id = 1; % Truck actor ID
ego_id = 2;   % Ego vehicle actor ID

% Initialize arrays to store the positions over time
egoPositions = []; % Initialize empty for ego positions

% Initialize the last truck lateral position
lastTruckLateralPos = NaN;

% Initialize flags to track truck's lane position
inSameLane = false;  % Track if truck is in the same lane as the ego vehicle
inLeftLane = false;  % Track if truck is in the left lane
inRightLane = false; % Track if truck is in the right lane

% Initialize variables to store the last known lane boundaries
lastLeftLaneBoundary = NaN;
lastRightLaneBoundary = NaN;

% Debugging information
fprintf('Estimated trajectory size: %d x %d\n', size(estimated_trajectory));

for t = 1:length(allData)
    % Get the lane boundaries detected by the camera sensor
    laneBoundaries = allData(t).LaneDetections.LaneBoundaries;

    % Check if laneBoundaries is empty or doesn't have at least 2 lanes
    if isempty(laneBoundaries) || length(laneBoundaries) < 2
        continue; % Skip this iteration if lane boundaries are not valid
    end

    % Get the ego vehicle's position
    egoPosition = allData(t).ActorPoses(ego_id).Position;     % Ego is actor 2

    % Store ego positions for later plotting
    egoPositions = [egoPositions; egoPosition]; %#ok<AGROW>

    % Extract lane boundaries lateral offsets
    leftLaneBoundary = laneBoundaries(1).LateralOffset;  % Left lane boundary
    rightLaneBoundary = laneBoundaries(2).LateralOffset; % Right lane boundary

    % Check for NaN values in lane boundaries
    if isnan(leftLaneBoundary) || isnan(rightLaneBoundary)
        leftLaneBoundary = lastLeftLaneBoundary;
        rightLaneBoundary = lastRightLaneBoundary;
    else
        lastLeftLaneBoundary = leftLaneBoundary;  % Update last known value
        lastRightLaneBoundary = rightLaneBoundary; % Update last known value
    end

    % Truck's lateral position relative to the lane (using the global y-coordinate)
    % Use the estimated trajectory for the truck's position
    if t <= size(estimated_trajectory, 1)
        truckLateralPos = estimated_trajectory(t, 2); % Use estimated trajectory for truck's y-coordinate
    else
        truckLateralPos = NaN; % Handle case if estimated trajectory has fewer entries
    end

    % Determine truck's current lane status
    currentlyInSameLane = truckLateralPos > rightLaneBoundary && truckLateralPos < leftLaneBoundary;
    currentlyInLeftLane = truckLateralPos > leftLaneBoundary;
    currentlyInRightLane = truckLateralPos < rightLaneBoundary;

    % Check if truck has entered the same lane as the ego vehicle
    if currentlyInSameLane
        if ~inSameLane  % If truck wasn't in the same lane before
            if inLeftLane
                disp(['Warning: Truck entered the same lane as ego vehicle at time = ', num2str(allData(t).Time), ' (Entered from left)']);
            elseif inRightLane
                disp(['Warning: Truck entered the same lane as ego vehicle at time = ', num2str(allData(t).Time), ' (Entered from right)']);
            end
            inSameLane = true;  % Update the flag
            inLeftLane = false;
            inRightLane = false;
        end
    else
        % Handle truck leaving the ego lane
        if inSameLane
            if lastTruckLateralPos > rightLaneBoundary && truckLateralPos < rightLaneBoundary
                disp(['Warning: Truck left the ego lane to the right lane at time = ', num2str(allData(t).Time)]);
            elseif lastTruckLateralPos < leftLaneBoundary && truckLateralPos > leftLaneBoundary
                disp(['Warning: Truck left the ego lane to the left lane at time = ', num2str(allData(t).Time)]);
            end
            inSameLane = false;
        end

        % Check if the truck is in a different lane (right or left)
        if currentlyInRightLane && ~inRightLane
            disp(['Truck identified in the right lane at time = ', num2str(allData(t).Time)]);
            inRightLane = true; 
            inLeftLane = false; 
        elseif currentlyInLeftLane && ~inLeftLane
            disp(['Truck identified in the left lane at time = ', num2str(allData(t).Time)]);
            inLeftLane = true; 
            inRightLane = false; 
        end
    end

    % Update the last truck lateral position for the next iteration
    lastTruckLateralPos = truckLateralPos;
end

% Plotting the lanes and vehicle trajectories
figure;
hold on;

% Define an x-range for plotting
xRange = [-100 150];  
plot(xRange, [lastLeftLaneBoundary lastLeftLaneBoundary], 'r--', 'DisplayName', 'Left Lane Boundary');  % Last known left lane boundary
plot(xRange, [lastRightLaneBoundary lastRightLaneBoundary], 'r--', 'DisplayName', 'Right Lane Boundary'); % Last known right lane boundary

% Plot ego vehicle trajectory (only x and y positions)
plot(egoPositions(:, 1), egoPositions(:, 2), 'b', 'DisplayName', 'Ego Vehicle');

% Plot truck trajectory (only x and y positions)
if size(estimated_trajectory, 1) >= length(allData)
    plot(estimated_trajectory(:, 1), estimated_trajectory(:, 2), 'g', 'DisplayName', 'Truck'); % Using estimated trajectory
else
    disp('Estimated trajectory does not cover all time steps. Adjusting plot to show available data.');
    plot(estimated_trajectory(:, 1), estimated_trajectory(:, 2), 'g', 'DisplayName', 'Truck'); % Plot available data
end

legend;
xlabel('X Position');
ylabel('Y Position');
title('Lane and Vehicle Positions');
hold off;


% %% working version without Kalman Filter
% clear;
% 
% % Call the simulation environment function to get the data
% [allData, ~, ~] = simulationEnvironment_FV_NN();
% 
% % Define the actor_id for which you want to get aggregated measures (e.g., ego vehicle with ID = 2)
% actor_id = 1;
% ego_id = 2;   
% 
% % Initialize arrays to store the positions over time
% truckPositions = [];
% egoPositions = [];
% 
% % Initialize the last truck lateral position
% lastTruckLateralPos = NaN;
% 
% % Initialize flags to track truck's lane position
% inSameLane = false;  % Track if truck is in the same lane as the ego vehicle
% inLeftLane = false;  % Track if truck is in the left lane
% inRightLane = false; % Track if truck is in the right lane
% 
% % Initialize variables to store the last known lane boundaries
% lastLeftLaneBoundary = NaN;
% lastRightLaneBoundary = NaN;
% 
% for t = 1:length(allData)
%     % Get the lane boundaries detected by the camera sensor
%     laneBoundaries = allData(t).LaneDetections.LaneBoundaries;
% 
%     % Check if laneBoundaries is empty or doesn't have at least 2 lanes
%     if isempty(laneBoundaries) || length(laneBoundaries) < 2
%         continue; % Skip this iteration if lane boundaries are not valid
%     end
% 
%     % Get the actor vehicle's (truck) position and ego vehicle's position
%     truckPosition = allData(t).ActorPoses(actor_id).Position; % Truck is actor 1
%     egoPosition = allData(t).ActorPoses(ego_id).Position;     % Ego is actor 2
% 
%     % Store positions for later plotting
%     truckPositions = [truckPositions; truckPosition]; %#ok<AGROW>
%     egoPositions = [egoPositions; egoPosition]; %#ok<AGROW>
% 
%     % Extract lane boundaries lateral offsets
%     leftLaneBoundary = laneBoundaries(1).LateralOffset;  % Left lane boundary
%     rightLaneBoundary = laneBoundaries(2).LateralOffset; % Right lane boundary
% 
%     % Check for NaN values in lane boundaries
%     if isnan(leftLaneBoundary) || isnan(rightLaneBoundary)
%         leftLaneBoundary = lastLeftLaneBoundary;
%         rightLaneBoundary = lastRightLaneBoundary;
%     else
%         lastLeftLaneBoundary = leftLaneBoundary;  % Update last known value
%         lastRightLaneBoundary = rightLaneBoundary; % Update last known value
%     end
% 
%     % Truck's lateral position relative to the lane (using the global y-coordinate)
%     truckLateralPos = truckPosition(2); % Truck's y-coordinate
% 
%     % Determine truck's current lane status
%     currentlyInSameLane = truckLateralPos > rightLaneBoundary && truckLateralPos < leftLaneBoundary;
%     currentlyInLeftLane = truckLateralPos > leftLaneBoundary;
%     currentlyInRightLane = truckLateralPos < rightLaneBoundary;
% 
%     % Check if truck has entered the same lane as the ego vehicle
%     if currentlyInSameLane
%         if ~inSameLane  % If truck wasn't in the same lane before
%             if inLeftLane
%                 disp(['Warning: Truck entered the same lane as ego vehicle at time = ', num2str(allData(t).Time), ' (Entered from left)']);
%             elseif inRightLane
%                 disp(['Warning: Truck entered the same lane as ego vehicle at time = ', num2str(allData(t).Time), ' (Entered from right)']);
%             end
%             inSameLane = true;  % Update the flag
%             inLeftLane = false;
%             inRightLane = false;
%         end
%     else
%         % Handle truck leaving the ego lane
%         if inSameLane
%             if lastTruckLateralPos > rightLaneBoundary && truckLateralPos < rightLaneBoundary
%                 disp(['Warning: Truck left the ego lane to the left at time = ', num2str(allData(t).Time)]);
%             elseif lastTruckLateralPos < leftLaneBoundary && truckLateralPos > leftLaneBoundary
%                 disp(['Warning: Truck left the ego lane to the right at time = ', num2str(allData(t).Time)]);
%             end
%             inSameLane = false;
%         end
% 
%         % Check if the truck is in a different lane (right or left)
%         if currentlyInRightLane && ~inRightLane
%             disp(['Truck identified in the right lane at time = ', num2str(allData(t).Time)]);
%             inRightLane = true; 
%             inLeftLane = false; 
%         elseif currentlyInLeftLane && ~inLeftLane
%             disp(['Truck identified in the left lane at time = ', num2str(allData(t).Time)]);
%             inLeftLane = true; 
%             inRightLane = false; 
%         end
%     end
% 
%     % Update the last truck lateral position for the next iteration
%     lastTruckLateralPos = truckLateralPos;
% end
% 
% % Plotting the lanes and vehicle trajectories
% figure;
% hold on;
% 
% % Define an x-range for plotting
% xRange = [-50 150];  
% plot(xRange, [lastLeftLaneBoundary lastLeftLaneBoundary], 'r--', 'DisplayName', 'Left Lane Boundary');  % Last known left lane boundary
% plot(xRange, [lastRightLaneBoundary lastRightLaneBoundary], 'r--', 'DisplayName', 'Right Lane Boundary'); % Last known right lane boundary
% 
% % Plot ego vehicle trajectory (only x and y positions)
% plot(egoPositions(:, 1), egoPositions(:, 2), 'b', 'DisplayName', 'Ego Vehicle');
% 
% % Plot truck trajectory (only x and y positions)
% plot(truckPositions(:, 1), truckPositions(:, 2), 'g', 'DisplayName', 'Truck');
% 
% legend;
% xlabel('X Position');
% ylabel('Y Position');
% title('Lane and Vehicle Positions');
% hold off;