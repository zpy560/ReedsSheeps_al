% clear;close all;clc;
Vehicle.WB = 3.1;  % [m] wheel base: rear to front steer
Vehicle.W = 2.3; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.7; % [rad] maximum steering angle
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius
x0 = -8.5;y0 = -4;
goalslot = [x0 y0 0];
slotx = [-12 x0-1 x0-1 x0+6 x0+6 5 5 -12 -12];
sloty = [y0+1.5 y0+1.5 y0-1.5 y0-1.5 y0+1.5 y0+1.5 3 3 y0+1.5];
path = FindRSPath(goalslot(1),goalslot(2),goalslot(3),Vehicle);
figure(10);
writerObj=VideoWriter('parking_parrel_in.avi'); %// 定义一个视频文件用来存动画
open(writerObj); %// 打开该视频文件
[Route_x,Route_y,Route_phi] = PlotPath(path,Vehicle);
Num = length(Route_x);
for i = 1:Num
    PlotVehicle(0,0,0);
    PlotVehicle(goalslot(1),goalslot(2),goalslot(3));
    PlotPath(path,Vehicle);
    plot(slotx,sloty,'Color','k');hold on;
    PlotCurVeh(Route_x(i),Route_y(i),Route_phi(i));
    axis equal;
    pause(0.01);
    frame = getframe; %// 把图像存入视频文件中
    frame.cdata = imresize(frame.cdata, [653 514]); %重新定义帧大小
    writeVideo(writerObj,frame); %// 将帧写入视频
    hold off;
end
close(writerObj); %// 关闭视频文件句柄
