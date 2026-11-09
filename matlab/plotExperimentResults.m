function plotExperimentResults(results,folder)
if nargin<2, folder=pwd; end
fig=figure('Name','Closed-loop stress tests','Color','w','Position',[100 100 1200 750]);
metrics={'meanCrossTrackError','meanReconstructionError','meanPlanningMs','success'};
labels={'Mean driving CTE (m)','Mean reconstruction error (m)','Planning time (ms)','Success rate'};
scenarios=unique(results.scenario,'stable');
for j=1:4
    ax=subplot(2,2,j,'Parent',fig); values=nan(numel(scenarios),3);
    for k=1:numel(scenarios)
        for strategy=1:3
            rows=strcmp(results.scenario,scenarios{k}) & results.strategy==strategy;
            data=double(results.(metrics{j})(rows)); data=data(isfinite(data));
            if ~isempty(data), values(k,strategy)=mean(data); end
        end
    end
    bar(ax,values); ylabel(ax,labels{j}); grid(ax,'on');
    set(ax,'XTick',1:numel(scenarios),'XTickLabel',scenarios,'XTickLabelRotation',65,'FontSize',7);
    if j==1, legend(ax,{'Basic','Geometry','Graph'},'Location','best'); end
end
saveas(fig,fullfile(folder,'stress_summary.png')); savefig(fig,fullfile(folder,'stress_summary.fig'));
groups={'noise','range','missing'}; xlabels={'Noise sigma (m)','Range (m)','Miss probability'};
if ~any(ismember(results.group,groups)), return; end
fig=figure('Name','Sensitivity sweeps','Color','w','Position',[120 120 1200 650]);
for j=1:3
    for metric=1:2
        ax=subplot(2,3,j+(metric-1)*3,'Parent',fig); hold(ax,'on');
        for strategy=1:3
            rows=strcmp(results.group,groups{j}) & results.strategy==strategy;
            x=unique(results.parameter(rows)); y=nan(size(x));
            for k=1:numel(x)
                mask=rows & results.parameter==x(k);
                if metric==1, data=double(results.success(mask));
                else, data=results.meanCrossTrackError(mask); end
                data=data(isfinite(data)); if ~isempty(data), y(k)=mean(data); end
            end
            plot(ax,x,y,'-o','LineWidth',1.3);
        end
        xlabel(ax,xlabels{j}); grid(ax,'on');
        if metric==1, ylabel(ax,'Success fraction'); ylim(ax,[0 1]);
        else, ylabel(ax,'Mean driving CTE (m)'); end
        if j==1 && metric==1, legend(ax,{'Basic','Geometry','Graph'},'Location','best'); end
    end
end
saveas(fig,fullfile(folder,'sensitivity.png')); savefig(fig,fullfile(folder,'sensitivity.fig'));
end
