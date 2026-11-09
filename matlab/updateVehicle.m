function v=updateVehicle(v,u,c,dt)
% Bicycle integration uses midpoint yaw and mean velocity; no waypoint jumps.
change=max(-c.steerRate*dt,min(c.steerRate*dt,u.steer-v.steer));
v.steer=max(-c.maxSteer,min(c.maxSteer,v.steer+change));
nextSpeed=max(0,v.speed+u.accel*dt); speed=0.5*(v.speed+nextSpeed);
rate=speed/c.wheelbase*tan(v.steer);
v.x=v.x+speed*cos(v.yaw+0.5*rate*dt)*dt;
v.y=v.y+speed*sin(v.yaw+0.5*rate*dt)*dt;
v.yaw=wrapAngle(v.yaw+rate*dt); v.speed=nextSpeed; v.accel=u.accel;
end
