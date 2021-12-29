clear;close all;clc;
Vehicle.WB = 3.7;  % [m] wheel base: rear to front steer
Vehicle.W = 2.3; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius
Startslot = [5 8 pi/2];
goalslot = [-1 0 0];
path = FindRSPath(Startslot(1),Startslot(2),Startslot(3),Vehicle);
% route_x=[];route_y=[];route_angle=[];%定义轨迹点数
PlotPath(path,Vehicle);
Num = length(path.type);
type = path.type;
x=[];y=[];angle=[];
rmin = Vehicle.MIN_CIRCLE;
seg = [path.t,path.u,path.v,path.w,path.x];
pvec = [0,0,0];
PlotVehicle(Startslot(1),Startslot(2),Startslot(3));
PlotVehicle(goalslot(1),goalslot(2),goalslot(3));
plot([1 1 -2.5 -2.5 1 1],[16 1 1 -1 -1 -5],'LineWidth',2,'Color','k');hold on;
plot([8 8],[16 -5],'LineWidth',2,'Color','k');
% axis equal;
axis ([-3 18 -5 16]);
pause(0.1);
hold off;
Currslot = Control_Point(n,:);
PlotVehicle(Currslot(1),Currslot(2),Currslot(3)+pi);
hold on;
% end
PlotVehicle(Startslot(1),Startslot(2),Startslot(3));
PlotVehicle(goalslot(1),goalslot(2),goalslot(3));
plot(Control_Point(:,1),Control_Point(:,2),'Color','k','LineWidth',2);
