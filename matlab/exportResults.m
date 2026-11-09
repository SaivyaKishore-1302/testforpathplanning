function exportResults(s,folder)
if nargin<2, folder=fullfile(pwd,'results'); end
if ~exist(folder,'dir'), mkdir(folder); end
result=summarizeRun(s); config=s.config; track=s.environment; log=s.log; %#ok<NASGU>
names={'time','x','y','speed','steering','crossTrackError','pathMeanError', ...
    'pathMaxError','pathRMSError','planningMs','validPath','observedCones', ...
    'mappedCones','selectedGates','validGateFraction','candidateEdges'};
writetable(array2table(log,'VariableNames',names),fullfile(folder,'telemetry.csv'));
writetable(struct2table(result),fullfile(folder,'summary.csv'));
save(fullfile(folder,'run.mat'),'result','config','track','log');
end
