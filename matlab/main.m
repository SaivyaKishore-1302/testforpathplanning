function main(c,track)
%MAIN Launch the Formula Student Delaunay research simulator.
% Start paused. Press Start; choose Tools > Settings for all numerical controls.
if nargin<1, c=defaultConfig(); end
if nargin<2, track=generateTrack(c); end
c.sim.stopOnPlannerFailure=false; % An interactive obstruction is a wait, not an exit.
state=initializeSimulation(c,track); c=state.config; running=false; busy=false;
fig=figure('Name','FORMULA STUDENT | Delaunay research simulator','NumberTitle','off', ...
    'Color','w','Position',[70 60 1450 900],'CloseRequestFcn',@closeApp, ...
    'DefaultUicontrolForegroundColor',[0.12 0.12 0.12], ...
    'DefaultUicontrolBackgroundColor',[0.94 0.94 0.94], ...
    'DefaultAxesXColor',[0.2 0.2 0.2],'DefaultAxesYColor',[0.2 0.2 0.2], ...
    'DefaultTextColor',[0.15 0.15 0.15]);
ax1=axes(fig,'Position',[0.045 0.43 0.61 0.49]);
ax2=axes(fig,'Position',[0.045 0.085 0.61 0.285]);
g1=[]; g2=[];
moveStage=0; moveIndex=[]; savedMoveCallbacks=struct();
moveMarker=line(ax1,nan,nan,'LineStyle','none','Marker','o','MarkerSize',15, ...
    'Color',[0.85 0.1 0.1],'LineWidth',2,'HitTest','off','PickableParts','none');
moveMessage=uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.045 0.395 0.61 0.026],'BackgroundColor','w', ...
    'ForegroundColor',[0.65 0.12 0.08],'HorizontalAlignment','left','FontSize',10);
uicontrol(fig,'Style','text','String','DELAUNAY / DRIVERLESS LAB','Units','normalized', ...
    'Position',[0.03 0.945 0.47 0.035],'BackgroundColor','w','FontSize',16, ...
    'FontWeight','bold','HorizontalAlignment','left');
scenarios=scenarioCatalog();
scenarioChoice=uicontrol(fig,'Style','popupmenu','String',scenarios,'Units','normalized', ...
    'Position',[0.51 0.947 0.32 0.032],'BackgroundColor','w');
button('Load scenario',0.84,0.945,0.12,@loadScenario);
types={'Oval','Figure eight','Technical','High speed','Random','Hairpin','Custom'};
trackChoice=popup('Track',types,0.91,@(~,~) []);
set(trackChoice,'Value',find(strcmpi(types,c.track.type),1));
strategyChoice=popup('Strategy',{'1 Basic','2 Geometry filtered','3 Gate graph'},0.85,@changeStrategy);
set(strategyChoice,'Value',c.planner.strategy);
modeChoice=popup('Perception',{'Perfect','Noisy','Memory'},0.79,@changeMode);
set(modeChoice,'Value',find(strcmpi({'Perfect','Noisy','Memory'},c.sensor.mode),1));
button('Generate',0.69,0.735,0.12,@newTrack); button('Draw / edit',0.82,0.735,0.14,@editTrack);
button('Start / Resume',0.69,0.68,0.17,@startRun); button('Pause',0.87,0.68,0.09,@pauseRun);
button('Single step',0.69,0.63,0.13,@singleStep); button('Reset',0.84,0.63,0.12,@resetRun);
uicontrol(fig,'Style','text','String','Animation speed (wall clock)','Units','normalized', ...
    'Position',[0.69 0.591 0.27 0.025],'BackgroundColor','w','HorizontalAlignment','left');
speed=uicontrol(fig,'Style','slider','Min',0.25,'Max',8,'Value',1, ...
    'Units','normalized','Position',[0.69 0.565 0.27 0.025]);
dashboard=uicontrol(fig,'Style','text','Units','normalized','Position',[0.685 0.195 0.3 0.355], ...
    'FontName','Consolas','FontSize',8,'BackgroundColor',[0.96 0.97 0.98], ...
    'HorizontalAlignment','left');
button('Settings',0.69,0.14,0.12,@settings); button('Export run',0.83,0.14,0.13,@exportRun);
button('Compare',0.69,0.09,0.12,@compare); button('Stress tests',0.83,0.09,0.13,@stress);
button('Move cone',0.69,0.045,0.12,@moveCone);
button('Noise / options',0.83,0.045,0.13,@hybridOptions);
uicontrol(fig,'Style','text','Units','normalized','Position',[0.68 0.002 0.31 0.035], ...
    'BackgroundColor','w','HorizontalAlignment','left','FontSize',9, ...
    'String',{'Green: path | Red x: anomaly | Cyan diamond: assumed', ...
    'Teal dotted: Voronoi | Grey reference: evaluation only'});
viewMenu=uimenu(fig,'Label','Layers');
fields=fieldnames(c.display);
for k=1:numel(fields)
    field=fields{k}; item=uimenu(viewMenu,'Label',field,'Checked',onoff(c.display.(field)));
    set(item,'Callback',@(src,~) toggleLayer(src,field));
end
fileMenu=uimenu(fig,'Label','Track files');
uimenu(fileMenu,'Label','Save layout','Callback',@saveTrack);
uimenu(fileMenu,'Label','Load layout','Callback',@loadTrack);
recoveryMenu=uimenu(fig,'Label','Cone recovery');
uimenu(recoveryMenu,'Label','Move physical cone (two clicks in GLOBAL view)','Callback',@moveCone);
uimenu(recoveryMenu,'Label','Toggle boundary recovery','Callback',@toggleRecovery);
clock=timer('ExecutionMode','fixedSpacing','Period',0.04,'BusyMode','drop', ...
    'TimerFcn',@tick,'ErrorFcn',@timerError);
render(); start(clock);

    function h=popup(label,items,y,callback)
        uicontrol(fig,'Style','text','String',label,'Units','normalized', ...
            'Position',[0.69 y 0.09 0.03],'BackgroundColor','w','HorizontalAlignment','left');
        h=uicontrol(fig,'Style','popupmenu','String',items,'Units','normalized', ...
            'Position',[0.78 y 0.18 0.032],'BackgroundColor','w','Callback',callback);
    end
    function button(label,x,y,w,callback)
        uicontrol(fig,'Style','pushbutton','String',label,'Units','normalized', ...
            'Position',[x y w 0.038],'FontSize',10,'Callback',callback);
    end
    function tick(~,~)
        if ~running || busy || ~isgraphics(fig), return; end
        busy=true; guard=onCleanup(@releaseBusy); %#ok<NASGU>
        count=max(1,round(0.04*get(speed,'Value')/c.sim.dt));
        frameClock=tic;
        try
            for ii=1:count
                state=stepSimulation(state);
                if state.done, running=false; break; end
                % Bound catch-up work so mouse/key callbacks get another turn
                % even when fast-forwarding through expensive hazard plans.
                if toc(frameClock)>=0.06, break; end
            end
            render();
        catch ex
            running=false; errordlg(getReport(ex,'basic','hyperlinks','off'),'Simulation error');
        end
    end
    function releaseBusy(), busy=false; end
    function render()
        g1=visualizeSimulation(ax1,g1,state,false); g2=visualizeSimulation(ax2,g2,state,true);
        if moveStage==2 && moveIndex<=size(state.environment.cones.xy,1)
            selected=state.environment.cones.xy(moveIndex,:);
            set(moveMarker,'XData',selected(1),'YData',selected(2));
        end
        v=state.vehicle; p=state.plan; e=state.evaluation; obs=state.observations;
        mapped=0; blue=0; yellow=0;
        if ~isempty(state.map)
            mapped=size(state.map.xy,1); blue=sum(state.map.colour==1); yellow=sum(state.map.colour==2);
        end
        uncertainty=0; if ~isempty(obs.sigma), uncertainty=mean(obs.sigma); end
        len=sum(vecnorm(diff(p.worldPath),2,2));
        message=sprintf(['VEHICLE\nX %7.2f  Y %7.2f m\nSpeed %.2f m/s | heading %.1f deg\n' ...
            'Steer %.1f deg | lap %d\nPERCEPTION\nObserved %d | mapped %d\nBlue %d | yellow %d | sigma %.2f m\n' ...
            'DELAUNAY\nVertices %d | triangles %d\nCandidates %d | selected %d\nRejected %d | midpoints %d\n' ...
            'PLANNING / EVALUATION\nPoints %d | path %.1f m | %.2f ms\nCTE %.3f m | reconstruction %.3f m\n' ...
            'Time %.1f s | distance %.1f m\nCollisions %d | boundary events %d\nComplete %d | %s'], ...
            v.x,v.y,v.speed,v.yaw*180/pi,v.steer*180/pi,e.lap,numel(obs.colour),mapped,blue,yellow, ...
            uncertainty,size(p.tri.points,1),size(p.tri.triangles,1),sum(p.filter.opposite), ...
            numel(p.route.edgeIds),size(p.tri.edges,1)-numel(p.route.edgeIds),sum(p.filter.valid), ...
            size(p.worldPath,1),len,p.time*1000,e.cte,e.pathMean,state.time,state.distance, ...
            e.collisionCount,e.violations,e.complete,state.reason);
        message=sprintf('%s\nAnomalies %d | assumed %d | brake %d\n%s | confidence %.2f',message, ...
            sum(p.recovery.anomaly),size(p.recovery.virtual,1)+size(p.boundary.virtual,1), ...
            state.safetyBraking,p.selected,p.confidence);
        if running, mode='RUNNING'; elseif state.done, mode='FINISHED'; else, mode='PAUSED'; end
        set(dashboard,'String',sprintf('%s\n%s',mode,message));
        setappdata(fig,'SimulationState',state); setappdata(fig,'SimulationRunning',running); drawnow limitrate
    end
    function startRun(~,~)
        if ~state.done, running=true; end
        render();
    end
    function loadScenario(~,~)
        if moveStage>0, finishMove('Move cancelled.'); end
        running=false;
        [proposed,env,meta]=makeScenario(scenarios{get(scenarioChoice,'Value')},c.seed);
        proposed.sim.scenario=meta; proposed.sim.stopOnPlannerFailure=false;
        state=initializeSimulation(proposed,env); c=state.config;
        set(trackChoice,'Value',find(strcmpi(types,c.track.type),1));
        set(modeChoice,'Value',3); set(strategyChoice,'Value',c.planner.strategy); render();
    end
    function hybridOptions(~,~)
        running=false;
        values=inputdlg({'Planner geometry noise std (m)','Random gate dropout (0 to 1)', ...
            'Sensor base noise std (m)','Sensor miss probability (0 to 1)', ...
            'Hybrid enabled (0 or 1)','Voronoi enabled (0 or 1)','Virtual boundary enabled (0 or 1)'}, ...
            'Hybrid planning options',[1 52],{num2str(c.planner.hybrid.planningNoise), ...
            num2str(c.planner.hybrid.planningDropout),num2str(c.sensor.sigma),num2str(c.sensor.missRate), ...
            num2str(c.planner.hybrid.enabled),num2str(c.planner.hybrid.voronoi),num2str(c.planner.hybrid.virtual)});
        if isempty(values), return; end
        v=str2double(values);
        if any(~isfinite(v)) || any(v<0) || any(v([2 4])>1) || any(~ismember(v(5:7),[0 1]))
            errordlg('Use nonnegative noise, probabilities in [0,1] and switches 0 or 1.'); return;
        end
        c.planner.hybrid.planningNoise=v(1); c.planner.hybrid.planningDropout=v(2);
        c.sensor.sigma=v(3); c.sensor.missRate=v(4);
        c.planner.hybrid.enabled=logical(v(5)); c.planner.hybrid.voronoi=logical(v(6));
        c.planner.hybrid.virtual=logical(v(7)); state.config=c; state.nextPlan=state.time; render();
    end
    function moveCone(~,~)
        if moveStage>0, finishMove('Move cancelled.'); return; end
        if isempty(state.environment.cones.xy)
            set(moveMessage,'String','There are no physical cones in this scene.'); return;
        end
        % Figure callbacks keep clicks scoped to the GLOBAL axes. Disable
        % built-in data tips/zoom so they cannot consume cone-selection clicks.
        savedMoveCallbacks=struct('down',get(fig,'WindowButtonDownFcn'), ...
            'key',get(fig,'WindowKeyPressFcn'),'pointer',get(fig,'Pointer'), ...
            'interactions',ax1.Interactions);
        pan(fig,'off'); zoom(fig,'off'); datacursormode(fig,'off');
        disableDefaultInteractivity(ax1);
        set(fig,'WindowButtonDownFcn',@moveClick,'WindowKeyPressFcn',@moveKey,'Pointer','crosshair');
        moveStage=1; moveIndex=[];
        set(moveMessage,'String','MOVE 1/2: click a cone in GLOBAL. The simulation keeps its running/paused state. Esc cancels.');
        setappdata(fig,'ConeMoveStage',moveStage); drawnow;
    end
    function moveClick(~,event)
        if moveStage==0 || ~strcmp(get(fig,'SelectionType'),'normal'), return; end
        % Read the mouse event's captured intersection, not axes CurrentPoint.
        % Animation can update CurrentPoint/current axes before a queued click
        % is handled. HitObject also belongs to the original click event.
        % WindowMouseData exposes these values through its accessor even on
        % releases where isprop/properties do not list them.
        try, hit=event.HitObject;
        catch, hit=hittest(fig); end
        if isempty(hit) || ~isgraphics(hit), return; end
        clickedAxes=ancestor(hit,'axes');
        if isequal(hit,ax1), clickedAxes=ax1; end
        if ~isequal(clickedAxes,ax1)
            set(moveMessage,'String','Use the upper GLOBAL view. Your selection is still active.'); return;
        end
        try
            xy=event.IntersectionPoint(1,1:2);
        catch
            point=get(ax1,'CurrentPoint'); xy=point(1,1:2);
        end
        if any(~isfinite(xy)) || xy(1)<ax1.XLim(1) || xy(1)>ax1.XLim(2) || ...
                xy(2)<ax1.YLim(1) || xy(2)>ax1.YLim(2), return; end
        if moveStage==1
            cones=state.environment.cones.xy;
            % Clicking a cone artist identifies its colour's actual physical
            % vertices directly; diagrams and FOV patches cannot steal hits.
            ids=(1:size(cones,1))';
            if isequal(hit,g1.blue), ids=find(state.environment.cones.colour==1);
            elseif isequal(hit,g1.yellow), ids=find(state.environment.cones.colour==2); end
            box=getpixelposition(ax1,true); ratio=diff(ax1.XLim)/diff(ax1.YLim);
            if box(3)/box(4)>ratio, box(3)=box(4)*ratio; else, box(4)=box(3)/ratio; end
            scale=[box(3)/diff(ax1.XLim),box(4)/diff(ax1.YLim)];
            [distance,j]=min(vecnorm((cones(ids,:)-xy).*scale,2,2));
            if isempty(j) || distance>24
                set(moveMessage,'String','No cone selected. Click closer to a blue or yellow triangle in GLOBAL.'); return;
            end
            index=ids(j);
            moveIndex=index; moveStage=2;
            set(moveMarker,'XData',cones(index,1),'YData',cones(index,2));
            set(moveMessage,'String',sprintf('MOVE 2/2: cone %d selected (red ring). Click its new position in GLOBAL.',index));
            setappdata(fig,'ConeMoveStage',moveStage); drawnow; return;
        end
        index=moveIndex; oldPosition=state.environment.cones.xy(index,:);
        state=movePhysicalCone(state,index,xy);
        if running, suffix='Running: sensor and planner will update.';
        else, suffix='Paused: press Start / Resume when ready.'; end
        finishMove(sprintf('Moved cone %d: (%.2f, %.2f) to (%.2f, %.2f) m. %s', ...
            index,oldPosition(1),oldPosition(2),xy(1),xy(2),suffix));
        render();
    end
    function moveKey(~,event)
        if ismember(event.Key,{'escape','return'}), finishMove('Move cancelled.'); end
    end
    function finishMove(message)
        if moveStage==0, return; end
        moveStage=0; moveIndex=[];
        set(fig,'WindowButtonDownFcn',savedMoveCallbacks.down, ...
            'WindowKeyPressFcn',savedMoveCallbacks.key,'Pointer',savedMoveCallbacks.pointer);
        ax1.Interactions=savedMoveCallbacks.interactions;
        set(moveMarker,'XData',nan,'YData',nan); set(moveMessage,'String',message);
        setappdata(fig,'ConeMoveStage',0);
    end
    function toggleRecovery(~,~)
        c.planner.recovery.enabled=~c.planner.recovery.enabled;
        state.config.planner.recovery=c.planner.recovery;
        state.nextPlan=state.time;
    end
    function pauseRun(~,~), running=false; render(); end
    function singleStep(~,~), running=false; state=stepSimulation(state); render(); end
    function resetRun(~,~)
        if moveStage>0, finishMove('Move cancelled.'); end
        running=false; env=state.environment;
        if isfield(state,'eventOriginalXY') && isfield(c.sim,'scenario')
            env.cones.xy(c.sim.scenario.eventIndex,:)=state.eventOriginalXY;
        end
        state=initializeSimulation(c,env); render();
    end
    function changeStrategy(src,~)
        c.planner.strategy=get(src,'Value'); state.config.planner=c.planner;
    end
    function changeMode(src,~)
        modes={'Perfect','Noisy','Memory'}; c.sensor.mode=modes{get(src,'Value')}; resetRun([],[]);
    end
    function newTrack(~,~)
        if moveStage>0, finishMove('Move cancelled.'); end
        running=false; proposed=c; proposed.track.type=types{get(trackChoice,'Value')};
        if strcmp(proposed.track.type,'Custom') && isempty(proposed.track.controls), editTrack([],[]); return; end
        try
            env=generateTrack(proposed); c=proposed; state=initializeSimulation(c,env); render();
        catch ex, errordlg(ex.message,'Invalid track'); end
    end
    function editTrack(~,~)
        if moveStage>0, finishMove('Move cancelled.'); end
        running=false; q=trackEditor(state.environment.controls/c.track.scale,c.track.closed);
        if isempty(q), return; end
        proposed=c; proposed.track.type='Custom'; proposed.track.controls=q;
        try
            env=generateTrack(proposed); c=proposed; set(trackChoice,'Value',7);
            state=initializeSimulation(c,env); render();
        catch ex, errordlg(ex.message,'Invalid track; previous layout preserved'); end
    end
    function toggleLayer(src,field)
        c.display.(field)=~c.display.(field); state.config.display=c.display;
        set(src,'Checked',onoff(c.display.(field))); render();
    end
    function settings(~,~)
        if moveStage>0, finishMove('Move cancelled.'); end
        running=false;
        [proposed,ok]=simulationSettings(c);
        if ~ok, return; end
        try
            env=generateTrack(proposed); c=proposed; state=initializeSimulation(c,env); render();
        catch ex, errordlg(ex.message,'Settings not applied'); end
    end
    function saveTrack(~,~)
        running=false; [name,folder]=uiputfile('*.mat','Save track'); if isequal(name,0), return; end
        track=state.environment; trackConfig=c.track; save(fullfile(folder,name),'track','trackConfig');
    end
    function loadTrack(~,~)
        if moveStage>0, finishMove('Move cancelled.'); end
        running=false; [name,folder]=uigetfile('*.mat','Load track'); if isequal(name,0), return; end
        try
            data=load(fullfile(folder,name),'track','trackConfig'); proposed=c;
            proposed.track=data.trackConfig; proposed.planner.width=proposed.track.width;
            validateTrack(data.track,proposed); fresh=initializeSimulation(proposed,data.track);
            c=proposed; state=fresh; set(trackChoice,'Value',find(strcmp(types,c.track.type),1)); render();
        catch ex, errordlg(ex.message,'Could not load layout'); end
    end
    function exportRun(~,~)
        running=false; folder=uigetdir(pwd,'Export telemetry and summary');
        if ~isequal(folder,0), exportResults(state,folder); end
    end
    function compare(~,~)
        running=false;
        try, compareStrategies(c,state.environment); catch ex, errordlg(ex.message); end
    end
    function stress(~,~)
        running=false;
        try, runHybridSuite(fullfile(pwd,'results','hybrid'),c.seed); catch ex, errordlg(ex.message); end
    end
    function timerError(~,event)
        running=false; warning('FS:Timer','Animation timer stopped: %s',event.Data.message);
    end
    function closeApp(~,~)
        running=false;
        if exist('clock','var') && isvalid(clock), stop(clock); delete(clock); end
        delete(fig);
    end
end

function value=onoff(flag)
if flag, value='on'; else, value='off'; end
end
