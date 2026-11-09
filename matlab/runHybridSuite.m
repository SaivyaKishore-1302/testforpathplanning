function results=runHybridSuite(folder,seeds,ids,mode)
%RUNHYBRIDSUITE Measure progress, conservative stops and failures separately.
% runHybridSuite('results/hybrid',[2685 2686]) runs every case twice.
if nargin<1, folder=fullfile(pwd,'results','hybrid'); end
if nargin<2, seeds=[2685 2686]; end
names=scenarioCatalog(); if nargin<3 || isempty(ids), ids=1:numel(names); end
if nargin<4, mode='hybrid'; end
if ~exist(folder,'dir'), mkdir(folder); end
rows=struct([]);
for seed=seeds(:)'
    for id=ids(:)'
        name=names{id}; fprintf('[%s] seed %d: %s ... ',mode,seed,name);
        row=struct('scenario',name,'seed',seed,'mode',mode,'expected','', ...
            'outcome','ERROR','passed',false,'progress',0,'distance',0,'elapsed',0, ...
            'collisions',0,'boundaryEvents',0,'maxCTE',0,'meanPlanMs',0, ...
            'voronoiSelections',0,'boundarySelections',0,'delaunaySelections',0, ...
            'escapeSelections',0,'virtualPlans',0,'anomalyPlans',0,'safetyBrakes',0,'reason','');
        try
            [c,track,meta]=makeScenario(name,seed); c.sim.scenario=meta;
            if strcmpi(mode,'delaunay')
                c.planner.hybrid.enabled=false; c.planner.hybrid.voronoi=false;
                c.planner.hybrid.virtual=false; c.planner.recovery.enabled=false;
            end
            s=initializeSimulation(c,track); row.expected=meta.expected;
            history=zeros(0,10); planTimes=[]; last=-inf;
            while ~s.done
                s=stepSimulation(s);
                if s.plan.stamp~=last
                    last=s.plan.stamp; planTimes(end+1,1)=1000*s.plan.time; %#ok<AGROW>
                    row.voronoiSelections=row.voronoiSelections+strcmp(s.plan.selected,'Voronoi');
                    row.boundarySelections=row.boundarySelections+strcmp(s.plan.selected,'Boundary matching');
                    row.delaunaySelections=row.delaunaySelections+strcmp(s.plan.selected,'Delaunay');
                    row.escapeSelections=row.escapeSelections+strcmp(s.plan.selected,'Curvature escape');
                    row.virtualPlans=row.virtualPlans+~isempty(s.plan.boundary.virtual);
                    row.anomalyPlans=row.anomalyPlans+any(s.plan.recovery.anomaly);
                    history(end+1,:)=[s.time,s.vehicle.x,s.vehicle.y,s.vehicle.speed, ...
                        s.evaluation.cte,s.plan.confidence,s.plan.valid,s.safetyBraking, ...
                        size(s.plan.boundary.virtual,1),sum(s.plan.recovery.anomaly)]; %#ok<AGROW>
                end
                row.safetyBrakes=row.safetyBrakes+s.safetyBraking;
                if ~strcmp(meta.expected,'stop') && s.evaluation.progress>=meta.targetProgress, break; end
                if strcmp(meta.expected,'stop') && isfinite(c.sensor.outageStart) && ...
                        s.time>c.sensor.outageStart+1 && ...
                        s.time<c.sensor.outageStart+c.sensor.outageDuration && s.vehicle.speed<0.05
                    break;
                end
            end
            e=s.evaluation; row.progress=e.progress; row.distance=s.distance;
            row.elapsed=s.time; row.collisions=e.collisionCount; row.boundaryEvents=e.violations;
            row.meanPlanMs=mean(planTimes); row.maxCTE=max(history(:,5)); row.reason=s.reason;
            if e.collisionCount>0, row.outcome='COLLISION';
            elseif e.violations>0, row.outcome='BOUNDARY EXIT';
            elseif strcmp(meta.expected,'lap') && e.complete, row.outcome='LAP COMPLETE'; row.passed=true;
            elseif strcmp(meta.expected,'stop') && s.vehicle.speed<0.05
                row.outcome='EXPECTED STOP'; row.passed=true;
            elseif ~strcmp(meta.expected,'stop') && e.progress>=meta.targetProgress
                row.outcome='SECTION PASSED'; row.passed=true;
            elseif s.vehicle.speed<0.05, row.outcome='UNRESOLVED STOP';
            else, row.outcome='TIME LIMIT'; end
            physical=s.environment.cones; trajectory=s.trajectory; lastPlan=s.plan; vehicle=s.vehicle;
            save(fullfile(folder,sprintf('case_%02d_seed_%d.mat',id,seed)), ...
                'row','meta','history','physical','trajectory','lastPlan','vehicle');
        catch ex
            row.reason=getReport(ex,'basic','hyperlinks','off');
        end
        fprintf('%s | %.1f m | collisions %d | %.1f ms\n',row.outcome,row.progress,row.collisions,row.meanPlanMs);
        if isempty(rows), rows=row; else, rows(end+1)=row; end %#ok<AGROW>
        results=struct2table(rows); writetable(results,fullfile(folder,'summary.csv'));
    end
end
save(fullfile(folder,'suite_results.mat'),'results');
fprintf('Passed objectives: %d/%d. Collision-free: %d/%d.\n', ...
    sum(results.passed),height(results),sum(results.collisions==0 & ~strcmp(results.outcome,'ERROR')),height(results));
end
