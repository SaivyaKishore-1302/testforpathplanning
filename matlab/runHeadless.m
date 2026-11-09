function [result,state]=runHeadless(c,track,recordFrames)
if nargin<1, c=defaultConfig(); end
if nargin<2, track=generateTrack(c); end
if nargin<3, recordFrames=false; end
state=initializeSimulation(c,track); state.recordFrames=recordFrames;
while ~state.done, state=stepSimulation(state); end
result=summarizeRun(state);
end
