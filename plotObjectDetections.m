function plotObjectDetections(allData)
    % Check if allData is empty
    if isempty(allData)
        error('The input allData is empty. Ensure check_checkv2() provides valid data.');
    end

    % Initialize figure for plotting object detections
    figure;
    hold on;
    grid on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Object Detections from Different Sensors');

    % Define colors and markers for each sensor type
    sensorStyles = {
        1, 'ro', 'Radar';       % Red circles for radar
        2, 'bs', 'Vision Left'; % Blue squares for left vision
        3, 'g^', 'Vision Right' % Green triangles for right vision
    };

    % Loop through each time step in allData
    for i = 1:numel(allData)
        detections = allData(i).ObjectDetections;

        % Process each detection in this time step
        for j = 1:numel(detections)
            detection = detections{j};
            sensorIndex = detection.SensorIndex;
            position = detection.Measurement(1:2); 
            
            % Find style for this sensor
            styleIdx = find([sensorStyles{:, 1}] == sensorIndex, 1);
            if ~isempty(styleIdx)
                plot(position(1), position(2), sensorStyles{styleIdx, 2}, ...
                    'DisplayName', sensorStyles{styleIdx, 3});
            end
        end
    end

    % Show legend only once per sensor
    legendNames = unique([sensorStyles(:, 3)]);
    for k = 1:numel(legendNames)
        plot(nan, nan, sensorStyles{k, 2}, 'DisplayName', legendNames{k});
    end
    legend show;
    hold off;
end
