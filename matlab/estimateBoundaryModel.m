function b=estimateBoundaryModel(recovery,c)
%ESTIMATEBOUNDARYMODEL Observed trace ordering, transverse matching and hypotheses.
% Virtual cones are regenerated from physical support every plan; never fed
% back into landmark tracking. They cannot recursively support more virtuals.
b=struct('left',zeros(0,2),'right',zeros(0,2),'virtual',zeros(0,2), ...
    'virtualColour',zeros(0,1),'virtualConfidence',zeros(0,1), ...
    'virtualSource',{{}},'centre',zeros(0,2),'confidence',0,'width',c.width, ...
    'usable',false,'oneSided',false);
p=recovery.corrected; colour=recovery.colour;
left=traceBoundary(p(colour==1,:),c); right=traceBoundary(p(colour==2,:),c);
b.left=left; b.right=right;
% Learn width only from approximately transverse, observed correspondences.
widths=[];
for side=1:2
    if side==1, own=left; other=right; sign=-1; else, own=right; other=left; sign=1; end
    for i=1:size(own,1)
        [t,n]=frame(own,i); %#ok<ASGLU>
        for j=1:size(other,1)-1
            [q,valid]=normalIntersection(own(i,:),sign*n,other(j,:),other(j+1,:));
            if valid
                w=norm(q-own(i,:));
                if w>c.hybrid.widthMin && w<c.hybrid.widthMax, widths(end+1)=w; end %#ok<AGROW>
            end
        end
    end
end
if numel(widths)>=3, b.width=0.65*median(widths)+0.35*c.width; end
centres=zeros(0,2); confidence=zeros(0,1);
for side=1:2
    if side==1, own=left; other=right; sign=-1; else, own=right; other=left; sign=1; end
    for i=1:size(own,1)
        if own(i,1)<-2, continue; end
        [~,n]=frame(own,i); across=sign*n; matched=false; best=inf; q=[];
        for j=1:size(other,1)-1
            if norm(other(j+1,:)-other(j,:))>c.hybrid.maxVirtualGap, continue; end
            [candidate,valid]=normalIntersection(own(i,:),across,other(j,:),other(j+1,:));
            w=norm(candidate-own(i,:));
            if valid && w>c.hybrid.widthMin && w<c.hybrid.widthMax && abs(w-b.width)<best
                q=candidate; best=abs(w-b.width); matched=true;
            end
        end
        if matched
            centres(end+1,:)=(own(i,:)+q)/2; confidence(end+1,1)=0.95; %#ok<AGROW>
        elseif c.hybrid.virtual && size(own,1)>=3 && own(i,1)<=c.hybrid.maxVirtualHorizon
            % Width-prior hypothesis, not a claim that the missing cone exists.
            q=own(i,:)+b.width*across;
            % Nearby real evidence takes precedence over an unbracketed
            % width-prior hypothesis; do not invent an inward extra boundary.
            if ~isempty(other) && min(vecnorm(other-q,2,2))<0.6*b.width, continue; end
            centres(end+1,:)=(own(i,:)+q)/2; confidence(end+1,1)=c.hybrid.virtualConfidence; %#ok<AGROW>
            b.virtual(end+1,:)=q; b.virtualColour(end+1,1)=3-side;
            b.virtualConfidence(end+1,1)=c.hybrid.virtualConfidence;
            b.virtualSource{end+1,1}='opposite-boundary width prior';
        end
    end
end
if ~isempty(b.virtual)
    b.left=traceBoundary([left;b.virtual(b.virtualColour==1,:)],c);
    b.right=traceBoundary([right;b.virtual(b.virtualColour==2,:)],c);
end
if size(centres,1)>=2
    [~,ids]=unique(round(centres/0.2),'rows','stable');
    centres=centres(ids,:); confidence=confidence(ids);
    b.centre=traceBoundary(centres,c);
    b.confidence=mean(confidence); b.usable=size(b.centre,1)>=2 && ...
        sum(vecnorm(diff(b.centre),2,2))>=c.minLength;
end
b.oneSided=size(left,1)<2 || size(right,1)<2;
if (~isempty(left) && min(vecnorm(left,2,2))>4) || (~isempty(right) && min(vecnorm(right,2,2))>4)
    b.confidence=min(b.confidence,0.6);
end
end

function [t,n]=frame(p,i)
if size(p,1)<2, t=[1 0];
elseif i==1, t=p(2,:)-p(1,:);
elseif i==size(p,1), t=p(end,:)-p(end-1,:);
else, t=p(i+1,:)-p(i-1,:); end
t=t/max(norm(t),eps); n=[-t(2) t(1)];
end
function [q,valid]=normalIntersection(origin,direction,a,b)
A=[direction',-(b-a)']; q=[nan nan]; valid=false;
if abs(det(A))<1e-6, return; end
tu=A\(a-origin)';
outside=max([0,-tu(2),tu(2)-1])*norm(b-a);
valid=tu(1)>0 && outside<=1.5;
q=origin+tu(1)*direction;
end
