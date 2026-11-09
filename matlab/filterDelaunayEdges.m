function f=filterDelaunayEdges(tri,c,strategy)
e=tri.edges; p=tri.points; colours=tri.colours;
f=struct('opposite',false(size(e,1),1),'valid',false(size(e,1),1), ...
    'midpoints',zeros(size(e,1),2),'direction',zeros(size(e,1),2), ...
    'length',zeros(size(e,1),1),'reason',{repmat({'Same colour'},size(e,1),1)}, ...
    'boundaries',zeros(0,2));
for i=1:size(e,1)
    a=e(i,1); b=e(i,2); width=norm(p(a,:)-p(b,:)); f.length(i)=width;
    f.midpoints(i,:)=(p(a,:)+p(b,:))/2;
    if colours(a)==colours(b)
        if width<c.maxGap*1.5, f.boundaries(end+1,:)=[a b]; end %#ok<AGROW>
        continue
    end
    f.opposite(i)=true;
    if colours(a)==2, temp=a; a=b; b=temp; end
    across=p(a,:)-p(b,:); f.direction(i,:)=[across(2),-across(1)]/max(width,eps);
    reason='Candidate';
    if strategy>1
        if width<c.minEdge || width>c.maxEdge
            reason='Edge length';
        elseif c.useWidth && (width<0.55*c.width || width>1.55*c.width)
            reason='Expected width';
        elseif c.useHeading && f.direction(i,1)<cos(c.maxHeadingError)
            reason='Heading incompatible';
        elseif f.midpoints(i,1)<-2
            reason='Behind vehicle';
        end
    end
    f.reason{i}=reason; f.valid(i)=strcmp(reason,'Candidate');
end
end
