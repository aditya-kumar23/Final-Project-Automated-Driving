function plotFusedData(fusedData)
    % Initialize arrays for plotting
    timeStamps = [fusedData.Time];
    fusedX = zeros(1, numel(fusedData));
    fusedY = zeros(1, numel(fusedData));
    
    radarX = [];
    radarY = [];
    vision1X = [];
    vision1Y = [];
    vision2X = [];
    vision2Y = [];

    % Extract fused data and individual sensor data for plotting
    for i = 1:numel(fusedData)
        % Extract the fusedData for X and Y positions
        fusedX(i) = fusedData(i).FusedEstimate.Filtered(1);
        fusedY(i) = fusedData(i).FusedEstimate.Filtered(2);
        
        % Extract individual sensor filtered measurements for plotting
        for j = 1:numel(fusedData(i).ObjectDetections)
            sensorData = fusedData(i).ObjectDetections{j};
            sensorIndex = sensorData.SensorIndex;
            measurement = sensorData.FilteredMeasurement;
            
            % Separate measurements based on sensor index
            switch sensorIndex
                case 1  % Radar sensor
                    radarX = [radarX, measurement(1)]; 
                    radarY = [radarY, measurement(2)]; 
                case 2  % Vision sensor 1
                    vision1X = [vision1X, measurement(1)]; 
                    vision1Y = [vision1Y, measurement(2)]; 
                case 3  % Vision sensor 2
                    vision2X = [vision2X, measurement(1)]; 
                    vision2Y = [vision2Y, measurement(2)]; 
            end
        end
    end

    % Plot the data
    figure;

    % Plot individual sensor data
    subplot(2, 1, 1);
    hold on;
    plot(radarX, radarY, 'ro', 'DisplayName', 'Radar');
    plot(vision1X, vision1Y, 'bo', 'DisplayName', 'Vision Sensor 1');
    plot(vision2X, vision2Y, 'mo', 'DisplayName', 'Vision Sensor 2');
    title('Individual Sensor Filtered Measurements');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend;
    grid on;
    hold off;

    % Plot fused and filtered data
    subplot(2, 1, 2);
    plot(fusedX, fusedY, 'k o', 'LineWidth', 1.5, 'DisplayName', 'Fused & Filtered');
    title('Fused and Filtered Data Over Time');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend;
    grid on;
end
