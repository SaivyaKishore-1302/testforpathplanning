function controls=trackEditor(existing,closed)
% Click to add, drag to reshape, right-click to delete; Done accepts edits.
if nargin<1, existing=zeros(0,2); end
if nargin<2, closed=true; end
controls=existing; dragging=0;
f=figure('Name','Track editor | click add, drag move, right-click delete', ...
    'NumberTitle','off','Color','w','Position',[180 120 950 700]);
ax=axes(f,'Position',[0.08 0.13 0.88 0.8]); axis(ax,'equal'); grid(ax,'on');
xlim(ax,[-50 50]); ylim(ax,[-40 40]); xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
title(ax,'At least 4 control points; avoid narrow loops and boundary overlaps');
h=line(ax,nan,nan,'Color',[0.2 0.35 0.45],'Marker','o','LineWidth',1.5, ...
    'MarkerFaceColor',[0.1 0.5 0.7],'HitTest','off');
uicontrol(f,'Style','pushbutton','String','Done','Units','normalized', ...
    'Position',[0.8 0.025 0.15 0.055],'Callback',@finish);
uicontrol(f,'Style','pushbutton','String','Clear','Units','normalized', ...
    'Position',[0.62 0.025 0.15 0.055],'Callback',@clearPoints);
set(f,'WindowButtonDownFcn',@down,'WindowButtonMotionFcn',@move, ...
    'WindowButtonUpFcn',@up,'CloseRequestFcn',@cancel);
redraw(); uiwait(f);
if isgraphics(f), delete(f); end

    function redraw()
        q=controls;
        if closed && size(q,1)>2, q=[q;q(1,:)]; end
        if isempty(q), set(h,'XData',nan,'YData',nan);
        else, set(h,'XData',q(:,1),'YData',q(:,2)); end
        drawnow limitrate
    end
    function down(~,~)
        object=hittest(f);
        if ~isequal(ancestor(object,'axes'),ax) && ~isequal(object,ax), return; end
        cp=get(ax,'CurrentPoint'); p=cp(1,1:2); distances=vecnorm(controls-p,2,2);
        [d,j]=min(distances);
        if strcmp(get(f,'SelectionType'),'alt')
            if ~isempty(d) && d<3, controls(j,:)=[]; end
        elseif ~isempty(d) && d<2
            dragging=j;
        else
            controls(end+1,:)=p; dragging=size(controls,1);
        end
        redraw();
    end
    function move(~,~)
        if dragging>0
            cp=get(ax,'CurrentPoint'); controls(dragging,:)=cp(1,1:2); redraw();
        end
    end
    function up(~,~), dragging=0; end
    function finish(~,~)
        if size(controls,1)<4, errordlg('Add at least four control points.'); return; end
        uiresume(f);
    end
    function clearPoints(~,~), controls=zeros(0,2); redraw(); end
    function cancel(~,~), controls=[]; uiresume(f); end
end
