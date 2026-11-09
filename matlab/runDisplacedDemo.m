function runDisplacedDemo()
%RUNDISPLACEDDEMO Launch the ORIGINAL animated UI with one displaced cone.
c=defaultConfig(); c.track.type='Custom'; c.track.closed=false;
c.track.controls=[0 0;20 0;40 0;60 0];
% Deterministic geometry for the first demonstration; Memory is still local
% sensor acquisition, not access to the generated nominal boundary.
c.sensor.mode='Memory'; c.sensor.sigma=0; c.sensor.distanceNoise=0;
c.sensor.missRate=0; c.sensor.colourAccuracy=1;
track=generateTrack(c);
[~,id]=min(vecnorm(track.cones.xy-[18 2],2,2));
track.cones.xy(id,2)=0.6;
main(c,track);
end
