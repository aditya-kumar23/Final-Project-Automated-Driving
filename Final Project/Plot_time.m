% Call the Sonnenstrasse_sim function to get the data
[allData, ~, ~] = Sonnenstrasse_sim();

% Extract time and point cloud data from allData
time = [allData.Time];
ptClouds = {allData.PointClouds};
if true
    % Plot the point clouds for each time step
    figure('Color', 'white'); % Set figure background to white
    for i = 1:numel(ptClouds)
        ptCloud = ptClouds{i};
        locations = ptCloud.Location;  % Get the locations
        indices = locations(:, 3) < 0.7; % Find indices where z-coordinate is less than 1
        locations(indices, 3) = 0;  % Set z-coordinate to zero for those indices

        % Create a new point cloud with modified locations
        %ptCloud = pointCloud(locations, 'Color', ptCloud.Color);  % Assuming 'Color' is another property you want to retain

        if ~isempty(ptCloud.Location)
            set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
            set(gca,'color','w');
            pcshow(ptCloud);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title(['LiDAR Measurements at Time: ', num2str(time(i))]);

            pause(0.1); % Pause to observe each time step
        end
    end
end