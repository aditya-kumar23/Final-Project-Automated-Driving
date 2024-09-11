function plotObjectDetections(allData)
    figure; % Create a new figure for plotting
    hold on; % Hold on to the current figure

    % Loop through all recorded data in allData
    for i = 1:length(allData)
        objectDetections = allData(i).ObjectDetections;
        
        % Loop through each detection in the current time step
        for j = 1:length(objectDetections)
            detection = objectDetections{j};
            % Extract the detected object's position
            pos = detection.Measurement(1:2); % Assuming 2D (x, y) position
            
            % Plot the position on the 2D graph
            plot(pos(1), pos(2), 'bo', 'MarkerSize', 8, 'DisplayName', 'Detected Object');
        end
    end
    
    % Customize plot
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Object Detections');
    grid on;
    legend show;
    hold off; % Release the current figure
end

