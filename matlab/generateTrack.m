function track = generateTrack(c,stream)
% Hidden generation geometry. The planner never accepts this structure.
if nargin<2, stream=RandStream('mt19937ar','Seed',c.seed); end
t=c.track; n=18; a=(0:n-1)'*2*pi/n;
switch lower(t.type)
    case 'oval'
        controls=[26*cos(a),17*sin(a)];
    case 'figure eight'
        a=(0:31)'*2*pi/32;
        controls=[30*sin(a),18*sin(2*a)];
    case 'technical'
        controls=[0 0;14 0;27 0;32 6;31 14;24 16;18 11;12 12; ...
            11 21;20 26;29 28;28 37;15 39;2 34;-6 25;-7 13;-5 4];
    case 'high speed'
        controls=[-35 -14;-15 -15;15 -15;35 -14;43 -7;43 7;35 14; ...
            15 15;-15 15;-35 14;-43 7;-43 -7];
    case 'hairpin'
        controls=[0 0;15 0;30 0;39 3;40 9;35 12;20 12;8 12; ...
            3 18;10 24;27 25;38 30;35 39;16 40;0 33;-7 20;-7 8];
    case 'random'
        rad=24+5*rand(stream,n,1);
        controls=[rad.*cos(a),0.8*rad.*sin(a)];
    case 'custom'
        controls=t.controls;
        if size(controls,1)<4, error('FS:Track','Custom tracks need at least four control points.'); end
    otherwise
        error('FS:Track','Unknown track type: %s',t.type);
end
if any(strcmpi(t.type,{'Technical','Hairpin'}))
    controls=1.6*controls; % Keep built-in tight turns compatible with a 4 m corridor.
end
controls=controls*t.scale;
if ~t.closed && ~strcmpi(t.type,'Custom'), controls=controls(1:ceil(end*0.75),:); end
if any(~isfinite(controls(:))) || any(vecnorm(diff(controls),2,2)<0.1)
    error('FS:Track','Control points must be finite and distinct.');
end
% Blend Catmull-Rom cubic and its chord; sharpness changes tangent strength.
ref=sampleCurve(controls,t.closed,t.sharpness);
arc=[0;cumsum(vecnorm(diff(ref),2,2))];
ss=(0:0.22:arc(end))'; ss=unique([ss;arc(end)]);
ref=interp1(arc,ref,ss,'linear');
dx=gradient(ref(:,1)); dy=gradient(ref(:,2));
tangent=[dx dy]./max(hypot(dx,dy),eps);
if t.closed
    forward=(ref(2,:)-ref(1,:))/norm(ref(2,:)-ref(1,:));
    backward=(ref(end,:)-ref(end-1,:))/norm(ref(end,:)-ref(end-1,:));
    seam=(forward+backward)/norm(forward+backward);
    tangent(1,:)=seam; tangent(end,:)=seam;
end
normal=[-tangent(:,2),tangent(:,1)];
width=t.width*(1+t.widthVariation*sin(2*pi*ss/ss(end)));
z=zeros(size(ss));
if strcmpi(t.type,'Figure eight')
    % Idealised overpass: observation height separates crossing corridors.
    phase=2*pi*ss/ss(end);
    z=4*(0.5+0.5*cos(phase)).*exp(-(sin(phase)/0.4).^2);
end
track=struct('reference',ref,'s',ss,'length',ss(end),'normal',normal, ...
    'width',width,'left',ref+normal.*width/2,'right',ref-normal.*width/2, ...
    'z',z,'closed',t.closed,'controls',controls,'type',t.type);
track=generateCones(track,c,stream);
validateTrack(track,c);
track.start=[ref(1,:),atan2(tangent(1,2),tangent(1,1))];
end

function p=sampleCurve(q,closed,sharpness)
N=size(q,1); p=zeros(0,2); tension=max(0.08,0.5*(1-sharpness));
if closed, segments=N; else, segments=N-1; end
for i=1:segments
    j=mod(i,N)+1;
    if closed
        pre=q(mod(i-2,N)+1,:); post=q(mod(i+1,N)+1,:);
    else
        pre=q(max(1,i-1),:); post=q(min(N,j+1),:);
    end
    m1=tension*(q(j,:)-pre); m2=tension*(post-q(i,:));
    u=(0:0.035:0.999)';
    p=[p;(2*u.^3-3*u.^2+1)*q(i,:)+(u.^3-2*u.^2+u)*m1+ ...
        (-2*u.^3+3*u.^2)*q(j,:)+(u.^3-u.^2)*m2]; %#ok<AGROW>
end
if closed, p=[p;q(1,:)]; else, p=[p;q(end,:)]; end
end
