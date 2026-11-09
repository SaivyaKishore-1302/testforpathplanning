function plotHybridResults(folder)
%PLOTHYBRIDRESULTS Plot measured outcomes and example physical trajectories.
if nargin<1, folder=fullfile(pwd,'results','hybrid'); end
results=readtable(fullfile(folder,'summary.csv'),'TextType','string');
names=unique(results.scenario,'stable'); seeds=unique(results.seed); values=zeros(numel(names),numel(seeds));
for i=1:numel(names)
    for j=1:numel(seeds)
        row=results(results.scenario==names(i) & results.seed==seeds(j),:);
        if ~isempty(row) && row.passed
            values(i,j)=1+strcmp(row.outcome,'EXPECTED STOP');
        end
    end
end
f=reportFigure([50 50 1200 1100]);
imagesc(values); colormap(f,[0.85 0.3 0.25;0.2 0.65 0.4;0.2 0.5 0.85]); clim([0 2]);
set(gca,'YTick',1:numel(names),'YTickLabel',names,'XTick',1:numel(seeds), ...
    'XTickLabel',compose('Seed %d',seeds),'TickLength',[0 0],'FontSize',11);
title(sprintf('Measured hybrid objectives: %d / %d passed',sum(results.passed),height(results)), ...
    'Color',[0.1 0.15 0.2],'FontSize',15);
bar=colorbar; bar.Color=[0.1 0.15 0.2]; bar.Ticks=[0 1 2];
bar.TickLabels={'Unresolved','Progress met','Expected stop'};
hold on; for j=1:numel(seeds)-1, xline(j+0.5,'w-','LineWidth',2); end
exportgraphics(f,fullfile(folder,'scenario_outcomes.png'),'Resolution',150); close(f);
f=reportFigure([50 50 1500 650]);
layout=tiledlayout(f,2,3,'TileSpacing','loose','Padding','loose'); ids=[2 7 9 12 20 32];
for i=1:numel(ids)
    data=load(fullfile(folder,sprintf('case_%02d_seed_%d.mat',ids(i),seeds(1))));
    ax=nexttile(layout); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
    p=data.physical;
    plot(ax,p.xy(p.colour==1,1),p.xy(p.colour==1,2),'^','Color',[0.1 0.35 0.85]);
    plot(ax,p.xy(p.colour==2,1),p.xy(p.colour==2,2),'^','Color',[0.85 0.65 0.05]);
    plot(ax,data.trajectory(:,1),data.trajectory(:,2),'k-','LineWidth',1.5);
    route=data.lastPlan.worldPath; plot(ax,route(:,1),route(:,2),'-','Color',[0.1 0.6 0.4]);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
    title(ax,{data.row.scenario,data.row.outcome},'Interpreter','none','FontSize',10,'Color',[0.1 0.15 0.2]);
    xlim(ax,[min(data.trajectory(:,1))-4,max(data.trajectory(:,1))+6]);
    ylim(ax,[min(data.trajectory(:,2))-5,max(data.trajectory(:,2))+5]);
end
exportgraphics(f,fullfile(folder,'example_trajectories.png'),'Resolution',150); close(f);
end

function f=reportFigure(position)
f=figure('Visible','off','Color','w','Position',position, ...
    'DefaultAxesColor','w','DefaultAxesXColor',[0.1 0.15 0.2], ...
    'DefaultAxesYColor',[0.1 0.15 0.2],'DefaultTextColor',[0.1 0.15 0.2], ...
    'DefaultAxesFontSize',10);
end
