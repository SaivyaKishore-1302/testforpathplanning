function track=generateCones(track,c,stream)
t=c.track; spacing=t.spacing;
if t.targetCones>0, spacing=2*track.length/max(8,t.targetCones); end
s=0; stations=zeros(0,1);
while s<track.length-0.3
    stations(end+1,1)=s; %#ok<AGROW>
    s=s+spacing*max(0.3,1+t.irregularity*(2*rand(stream)-1));
end
if track.closed && numel(stations)>2 && track.length-stations(end)<0.65*spacing
    stations(end)=[]; % Do not create an overlapping cone pair at the closed seam.
end
if ~track.closed, stations=unique([stations;track.length]); end
left=interp1(track.s,track.left,stations); right=interp1(track.s,track.right,stations);
z=interp1(track.s,track.z,stations);
keepB=rand(stream,numel(stations),1)>=t.missingBlue;
keepY=rand(stream,numel(stations),1)>=t.missingYellow;
track.cones=struct('xy',[left(keepB,:);right(keepY,:)], ...
    'colour',[ones(sum(keepB),1);2*ones(sum(keepY),1)], ...
    'z',[z(keepB);z(keepY)],'station',[stations(keepB);stations(keepY)]);
end
