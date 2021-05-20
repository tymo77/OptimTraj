function writeSTOFiles(soln, info, resultDir)

ControlsData.data = [soln.grid.time.' soln.grid.control.'];
ControlsData.name = [resultDir 'sol_controls'];
ControlsData.labels = ['time' info.controls_names'];
ControlsData.nRows = size(ControlsData.data, 1);
ControlsData.nColumns = size(ControlsData.data, 2);
ControlsData.inDegrees = 0;

writeOpenSimControlFile(ControlsData);

StatesData.data = [soln.grid.time.' soln.grid.state.'];
StatesData.name = [resultDir 'sol_states'];
StatesData.labels = ['time' info.states_active_names'];
StatesData.nRows = size(StatesData.data, 1);
StatesData.nColumns = size(StatesData.data, 2);
StatesData.inDegrees = 0;

writeOpenSimStatesFile(StatesData);

end