[allData, scenario, sensors] = check_checkv2();
fusedData = multisensorFilter_fusion_Adaptiv_weight();
plotActorTrajectories(allData);
plotObjectDetections(allData);
plotFusedData(fusedData);
