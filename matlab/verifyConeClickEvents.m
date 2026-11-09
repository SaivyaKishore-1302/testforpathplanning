function verifyConeClickEvents()
%VERIFYCONECLICKEVENTS Regression: live redraws must not replace click coordinates.
main(); fig=findall(groot,'Type','figure','Name','FORMULA STUDENT | Delaunay research simulator');
cleanup=onCleanup(@() close(fig)); %#ok<NASGU>
blue=findall(fig,'Tag','PhysicalBlueCones'); ax=ancestor(blue,'axes');
assert(isscalar(blue) && strcmp(blue.PickableParts,'visible'));
objects=findall(ax,'Type','line','-or','Type','patch');
for k=1:numel(objects)
    if ~ismember(objects(k).Tag,{'PhysicalBlueCones','PhysicalYellowCones'})
        assert(strcmp(objects(k).PickableParts,'none'),'A planning layer can intercept cone clicks.');
    end
end
start=findall(fig,'Style','pushbutton','String','Start / Resume'); callback=get(start,'Callback'); callback(start,[]);
pause(0.8);
move=findall(fig,'Style','pushbutton','String','Move cone'); callback=get(move,'Callback'); callback(move,[]);
before=getappdata(fig,'SimulationState'); ids=find(before.environment.cones.colour==1); index=ids(3);
position=before.environment.cones.xy(index,:);
% Render the local view/current axes between input capture and dispatch.
local=findall(fig,'Type','axes'); local=local(local~=ax); axes(local); %#ok<LAXES>
click=get(fig,'WindowButtonDownFcn');
click(fig,struct('HitObject',blue,'IntersectionPoint',[position 0]));
assert(getappdata(fig,'ConeMoveStage')==2,'Captured cone click was rejected after a redraw.');
destination=position+[1.2 0.8];
click(fig,struct('HitObject',ax,'IntersectionPoint',[destination 0]));
after=getappdata(fig,'SimulationState');
assert(norm(after.environment.cones.xy(index,:)-destination)<1e-8);
assert(getappdata(fig,'SimulationRunning'),'Placement paused the running simulation.');
assert(~isempty(after.map.xy),'Placement discarded map history.');
assert(getappdata(fig,'ConeMoveStage')==0);
fprintf('Captured click selection/placement after redraw, live running state and non-pickable overlays passed.\n');
end
