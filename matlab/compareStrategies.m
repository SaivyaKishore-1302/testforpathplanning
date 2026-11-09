function [summary,replay]=compareStrategies(c,track,frames,folder)
% Frozen-observation comparison. All strategies receive identical maps/poses.
% Driving statistics belong ONLY to the driver that recorded the frames.
if nargin<1, c=defaultConfig(); end
if nargin<2, track=generateTrack(c); end
if nargin<4, folder=fullfile(pwd,'results','comparison'); end
if ~exist(folder,'dir'), mkdir(folder); end
driver=[];
if nargin<3 || isempty(frames)
    recordingConfig=c; recordingConfig.planner.strategy=3;
    fprintf('Recording observations using strategy 3 ...\n');
    [driver,state]=runHeadless(recordingConfig,track,true); frames=state.frames;
end
replay=cell(3,1); rows=struct([]);
for strategy=1:3
    pc=c.planner; pc.strategy=strategy; previous=zeros(0,2); log=zeros(numel(frames),7);
    for i=1:numel(frames)
        frame=frames{i}; plan=planPath(frame.map,frame.pose,frame.z,pc,previous);
        previous=plan.worldPath;
        error=[];
        if plan.valid && size(plan.worldPath,1)>1
            ss=[0;cumsum(vecnorm(diff(plan.worldPath),2,2))];
            samples=interp1(ss,plan.worldPath,unique([(0:0.25:ss(end))';ss(end)]));
            % Match evaluation reference by deck before spatial projection.
            usable=abs(track.z-frame.z)<1;
            starts=find(usable(1:end-1)&usable(2:end));
            error=inf(size(samples,1),1);
            for j=starts'
                error=min(error,projectPolyline(samples,track.reference(j:j+1,:)));
            end
        end
        if isempty(error), avg=NaN; rms=NaN; mx=NaN;
        else, avg=mean(error); rms=sqrt(mean(error.^2)); mx=max(error); end
        log(i,:)=[frame.time,double(plan.valid),avg,rms,mx,plan.time*1000,numel(plan.route.edgeIds)];
    end
    replay{strategy}=array2table(log,'VariableNames', ...
        {'time','valid','meanError','rmsError','maxError','planningMs','selectedGates'});
    row=struct('strategy',strategy,'frames',numel(frames),'validFraction',mean(log(:,2)), ...
        'meanReconstructionError',finiteMean(log(:,3)),'meanPlanningMs',mean(log(:,6)));
    rows=[rows;row]; %#ok<AGROW>
    writetable(replay{strategy},fullfile(folder,sprintf('strategy_%d_replay.csv',strategy)));
end
summary=struct2table(rows); writetable(summary,fullfile(folder,'paired_summary.csv'));
save(fullfile(folder,'paired_replay.mat'),'summary','replay','driver','c','frames');
fig=figure('Color','w','Name','Identical-observation strategy comparison');
ax=subplot(2,1,1,'Parent',fig); hold(ax,'on');
for k=1:3, plot(ax,replay{k}.time,replay{k}.meanError,'LineWidth',1); end
ylabel(ax,'Mean path error (m)'); grid(ax,'on'); legend(ax,{'Basic','Geometry','Graph'});
title(ax,'Paired replay: identical observations, poses and mapping');
ax=subplot(2,1,2,'Parent',fig); bar(ax,summary.strategy,summary.validFraction);
xlabel(ax,'Strategy'); ylabel(ax,'Valid plan fraction'); ylim(ax,[0 1]); grid(ax,'on');
saveas(fig,fullfile(folder,'paired_comparison.png')); savefig(fig,fullfile(folder,'paired_comparison.fig'));
disp(summary);
end

function x=finiteMean(x)
x=x(isfinite(x)); if isempty(x), x=NaN; else, x=mean(x); end
end
