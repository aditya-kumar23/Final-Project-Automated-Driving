[allData, scenario, sensors] = check_checkv2();
fusedData = multisensorFilter_fusion_Adaptiv_weight();
plotActorTrajectories(allData);
plotObjectDetections(allData);
plotFusedData(fusedData);
ACC_CAS_System_Filtered_FusedData(fusedData, 35, 2, 25, 5, 1.2, 7, 1.2)
ACC_CAS_System_RawData()