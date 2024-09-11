% Call the Sonnenstrasse_sim function to get the data
[allData, scenario, sensors, egoVehicle] = Sonnenstrasse_sim();

%disp(scenario);
%disp(sensors);
plot(scenario)

while advance(scenario)
    pause(scenario.SampleTime)
end
 sensorR = sensors{1,2}; % Radar sensor information

bep = birdsEyePlot('XLim',[0 80],'YLim',[-35 35]);

lmPlotter = laneMarkingPlotter(bep,'Tag','lm','DisplayName','Lane markings');
olPlotter = outlinePlotter(bep,'Tag','ol');
caPlotter = coverageAreaPlotter(bep, ...
    'Tag','ca', ...
    'DisplayName','Radar coverage area', ...
    'FaceColor','red','EdgeColor','red');

%helperPlotScenario(bep,sensorR,egoVehicle)

clusterDetPlotter = detectionPlotter(bep, ...
    'DisplayName','Clustered detections', ...
    'MarkerEdgeColor','red', ...
    'MarkerFaceColor','red');


% sensorR.TargetReportFormat = "Detections";
 % detPlotter = detectionPlotter(bep, ...
 %     'DisplayName','Unclustered detections', ...
 %     'MarkerEdgeColor','red');

restart(scenario)
while advance(scenario)

    simTime = scenario.SimulationTime;
    targets = targetPoses(egoVehicle);
    [dets,numDets,isValidTime] = sensorR(targets,simTime);

    helperPlotScenario(bep,sensorR,egoVehicle)

    if isValidTime && numDets > 0
        detPos = cell2mat(cellfun(@(d)d.Measurement(1:2),dets, ...
            'UniformOutput',false)')';
        plotDetection(clusterDetPlotter,detPos)
    end

end

function helperPlotScenario(bep,radar,ego)

    % Plot lane markings
    lmPlotter = findPlotter(bep,'Tag','lm');
    [lmv,lmf] = laneMarkingVertices(ego);
    plotLaneMarking(lmPlotter,lmv,lmf)

    % Plot vehicle outlines
    olPlotter = findPlotter(bep,'Tag','ol');
    [position,yaw,length,width,originOffset,color] = targetOutlines(ego);
    plotOutline(olPlotter,position,yaw,length,width, ...
                   'OriginOffset',originOffset,'Color',color)

    % Plot radar coverage area
    caPlotter = findPlotter(bep,'Tag','ca');
    plotCoverageArea(caPlotter,radar.MountingLocation(1:2), ...
        radar.RangeLimits(2),radar.MountingAngles(1), ...
        radar.FieldOfView(1))

end

