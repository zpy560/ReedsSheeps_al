function [XA,XB,XC,XD,YA,YB,YC,YD,xrw,yrw,xfw,yfw] = PlotCurVeh(x,y,phi) % x,y 车辆中心点坐标，phi 车辆航向角（车辆前后中轴线与坐标系X方向夹角）
    
    % 车辆参数
    L = 4.5; %车长 m
    l = 3.1; %轴距 m
    Lw = 2; %轮距
    W = 2.3; %车宽
    
    x = x+l*cos(phi)/2;
    y = y+l*sin(phi)/2;
    
    % 前轴中心点坐标fw
    xfw = x + l*cos(phi)/2;
    yfw = y + l*sin(phi)/2;
    % 后轴中心点坐标rw
    xrw = x - l*cos(phi)/2;
    yrw = y - l*sin(phi)/2;
    % 车辆四轮与地面交点
    xa = x - l*cos(phi)/2 + Lw*sin(phi)/2; % 右后轮与地面交点坐标
    ya = y - l*sin(phi)/2 - Lw*cos(phi)/2;
    xb = x + l*cos(phi)/2 + Lw*sin(phi)/2; % 右前轮与地面交点坐标
    yb = y + l*sin(phi)/2 - Lw*cos(phi)/2;
    xc = x + l*cos(phi)/2 - Lw*sin(phi)/2; % 左前轮与地面交点坐标
    yc = y + l*sin(phi)/2 + Lw*cos(phi)/2;
    xd = x - l*cos(phi)/2 - Lw*sin(phi)/2; % 左后轮与地面交点坐标
    yd = y - l*sin(phi)/2 + Lw*cos(phi)/2;
    % 车身四个顶点坐标
    XA = x - L*cos(phi)/2 + W*sin(phi)/2; % 车辆右后顶点
    YA = y - L*sin(phi)/2 - W*cos(phi)/2;
    XB = x + L*cos(phi)/2 + W*sin(phi)/2; % 车辆右前顶点
    YB = y + L*sin(phi)/2 - W*cos(phi)/2;
    XC = x + L*cos(phi)/2 - W*sin(phi)/2; % 车辆左前顶点
    YC = y + L*sin(phi)/2 + W*cos(phi)/2;
    XD = x - L*cos(phi)/2 - W*sin(phi)/2; % 车辆左后顶点
    YD = y - L*sin(phi)/2 + W*cos(phi)/2;
    ABX = [XA XB];
    ABY = [YA YB];
    BCX = [XB XC];
    BCY = [YB YC];
    CDX = [XC XD];
    CDY = [YC YD];
    DAX = [XD XA];
    DAY = [YD YA];
    bcx = [xb xc];
    bcy = [yb yc];
    dax = [xd xa];
    day = [yd ya];
    frx = [xfw xrw];
    fry = [yfw yrw];
    plot(ABX,ABY,BCX,BCY,CDX,CDY,DAX,DAY,bcx,bcy,dax,day,frx,fry,'LineWidth',1.5,'Color','k');hold on;
%     plot([-5 -1.5 -1.5 6 6 15],[1 1 -1.2 -1.2 1 1],'LineWidth',2,'Color','k');
%     plot([-5 -3 -3 4 4 15],[1 1 -2 -2 1 1]);
%     axis([-5 15 -5 8]);
end