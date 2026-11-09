function verifyRecoveryUI()
%VERIFYRECOVERYUI Exercise original controls and added layers/callbacks.
main();
fig=findall(groot,'Type','figure','Name','FORMULA STUDENT | Delaunay research simulator');
cleanup=onCleanup(@() close(fig)); %#ok<NASGU>
step=findall(fig,'Style','pushbutton','String','Single step');
callback=get(step,'Callback'); callback(step,[]);
assert(numel(findall(fig,'Type','axes'))==2,'Original two views missing.');
assert(~isempty(findall(fig,'Type','patch')),'Animated vehicle missing.');
menu=findall(fig,'Type','uimenu','Label','Toggle boundary recovery');
callback=get(menu,'Callback'); callback(menu,[]); callback(menu,[]);
reset=findall(fig,'Style','pushbutton','String','Reset');
callback=get(reset,'Callback'); callback(reset,[]);
% New test controls must be visible without relying on MATLAB's menu chrome.
assert(~isempty(findall(fig,'Style','pushbutton','String','Move cone')));
moveButton=findall(fig,'Style','pushbutton','String','Move cone');
moveCallback=get(moveButton,'Callback'); moveCallback(moveButton,[]);
assert(getappdata(fig,'ConeMoveStage')==1,'Move mode failed to activate.');
keyCallback=get(fig,'WindowKeyPressFcn'); keyCallback(fig,struct('Key','escape'));
assert(getappdata(fig,'ConeMoveStage')==0,'Escape failed to cancel move mode.');
assert(isempty(get(fig,'WindowButtonDownFcn')),'Move mode did not restore the mouse callback.');
moveCallback(moveButton,[]); callback=get(reset,'Callback'); callback(reset,[]);
assert(getappdata(fig,'ConeMoveStage')==0,'Reset failed to cancel move mode.');
assert(~isempty(findall(fig,'Style','pushbutton','String','Noise / options')));
popups=findall(fig,'Style','popupmenu'); scenario=[];
for k=1:numel(popups)
    items=get(popups(k),'String');
    if numel(items)==numel(scenarioCatalog()), scenario=popups(k); break; end
end
assert(~isempty(scenario),'Scenario selector missing.');
set(scenario,'Value',2);
loadButton=findall(fig,'Style','pushbutton','String','Load scenario');
callback=get(loadButton,'Callback'); callback(loadButton,[]);
current=getappdata(fig,'SimulationState');
assert(strcmp(current.config.sim.scenario.name,'Staggered sides'));
startButton=findall(fig,'Style','pushbutton','String','Start / Resume');
callback=get(startButton,'Callback'); callback(startButton,[]); pause(0.8);
runningState=getappdata(fig,'SimulationState');
moveCallback(moveButton,[]);
assert(getappdata(fig,'SimulationRunning'),'Move cone paused the running simulation.');
assert(getappdata(fig,'ConeMoveStage')==1);
pause(1.2); liveState=getappdata(fig,'SimulationState');
assert(liveState.time>runningState.time,'Animation stopped while selecting a cone.');
keyCallback=get(fig,'WindowKeyPressFcn'); keyCallback(fig,struct('Key','escape'));
assert(getappdata(fig,'SimulationRunning'),'Cancelling cone selection paused the simulation.');
pauseButton=findall(fig,'Style','pushbutton','String','Pause');
callback=get(pauseButton,'Callback'); callback(pauseButton,[]);
current=getappdata(fig,'SimulationState'); assert(current.time>0);
assert(~getappdata(fig,'SimulationRunning'));
moveCallback(moveButton,[]);
assert(~getappdata(fig,'SimulationRunning'),'Move cone resumed a paused simulation.');
keyCallback=get(fig,'WindowKeyPressFcn'); keyCallback(fig,struct('Key','escape'));
% Resetting a dynamic case must replay one displacement, not accumulate kicks.
set(scenario,'Value',33); callback=get(loadButton,'Callback'); callback(loadButton,[]);
initial=getappdata(fig,'SimulationState'); id=initial.config.sim.scenario.eventIndex;
stepCallback=get(step,'Callback');
for k=1:102, stepCallback(step,[]); end
moved=getappdata(fig,'SimulationState'); assert(isfield(moved,'eventApplied'));
resetCallback=get(reset,'Callback'); resetCallback(reset,[]);
restored=getappdata(fig,'SimulationState');
assert(isequal(restored.environment.cones.xy(id,:),initial.environment.cones.xy(id,:)));
set(scenario,'Value',2); callback=get(loadButton,'Callback'); callback(loadButton,[]);
for k=1:20, stepCallback(step,[]); end
drawnow; frame=getframe(fig); imwrite(frame.cdata,'verification/updated_ui.png');
fprintf('UI, animated timer, reset, stepping, scenario load, visible controls and recovery toggle passed.\n');
end
