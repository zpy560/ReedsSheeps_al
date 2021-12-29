%% 起点坐标（9，4）
S = 9;
H = 4;
% theta = 0.6435; %0.8346;
%% 起点坐标（9，3）
% S = 9;
% H = 3;
% theta =0.6435; % 0.449;
%%
goalPose = [0,0,pi];
StartPose = [S,H,pi];
Rmin = 6;
syms x;
eqn = ((H - 2*Rmin)^2 + S^2)*x^2 + (4*(H - 2*Rmin)*Rmin)*x + 4*Rmin^2-S^2 == 0;
s = solve(eqn);
if (double(s(1)) > double(s(2)))
    theta = acos(double(s(1)));
else
    theta = acos(double(s(2)));
end
% 规划轨迹点
R1 = Rmin;
R2 = Rmin;
thetaR0 = 0.01;xR0 = S;yR0 = H;
thetaR1 = theta/5;
xR1 = S - R1*sin(thetaR1);
yR1 = H - R1*(1-cos(thetaR1));
thetaR2 = theta/4;
xR2 = S - R1*sin(thetaR2);
yR2 = H - R1*(1-cos(thetaR2));
thetaR3 = theta/3;
xR3 = S - R1*sin(thetaR3);
yR3 = H - R1*(1-cos(thetaR3));
thetaR4 = theta/2;
xR4 = S - R1*sin(thetaR4);
yR4 = H - R1*(1-cos(thetaR4));

xLine1 = S - R1*sin(theta);
yLine1 =  H - R1*(1-cos(theta));
xLine5 = R2 *sin(theta);
yLine5 = R2 *(1 - cos(theta));
xLine3 = (xLine1 + xLine5)/2;
yLine3 = tan(theta)*(xLine3 - R2 *sin(theta)) + R2 *(1 - cos(theta));
xLine2 = (xLine3 + xLine1)/2;
yLine2 = tan(theta)*(xLine2 - R2 *sin(theta)) + R2 *(1 - cos(theta));
xLine4 = (xLine5 + xLine3)/2;
yLine4 = tan(theta)*(xLine4 - R2 *sin(theta)) + R2 *(1 - cos(theta));

thetaR5 = theta/5;
xR5 = R2 *sin(thetaR5);
yR5 = R2 *(1 - cos(thetaR5));
thetaR6 = theta/4;
xR6 = R2 *sin(thetaR6);
yR6 = R2 *(1 - cos(thetaR6));
thetaR7 = theta/3;
xR7 = R2 *sin(thetaR7);
yR7 = R2 *(1 - cos(thetaR7));
thetaR8 = theta/2;
xR8 = R2 *sin(thetaR8);
yR8 = R2 *(1 - cos(thetaR8));

thetaR9 = 0.01;xR9 = 0;yR9 = 0;
Route1 = [xR0,yR0,thetaR0;...
    xR1,yR1,thetaR1;...
    xR2,yR2,thetaR2;...
    xR3,yR3,thetaR3;...
    xR4,yR4,thetaR4;...
    xLine1,yLine1,theta;...
    xLine2,yLine2,theta;...
    xLine3,yLine3,theta;...
    xLine4,yLine4,theta;...
    xLine5,yLine5,theta;...
    xR8,yR8,thetaR8;...
    xR7,yR7,thetaR7;...
    xR6,yR6,thetaR6;...
    xR5,yR5,thetaR5;...
    xR9,yR9,thetaR9;
    ];

% B样条曲线轨迹点
n = 15; k = 5;
P= Route1(1:n,1:2)' ;
% NodeVector = linspace(0, 1, n+k+1); % 均匀B样条的节点矢量
NodeVector = [0,0,0,0,0,0,1,2,3,4,5,6,7,8,9,10,10,10,10,10,10];
figure();
% 绘制样条曲线
plot(P(1, 1:n), P(2, 1:n),...
    'o','LineWidth',1,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','g',...
    'MarkerSize',6);
line(P(1, 1:n), P(2, 1:n));hold on;
Nik = zeros(n, 1);
umin =0;umax = 10;delta_u = 0.02;
Num = floor((umax-umin)/delta_u)+1;
Dest = zeros(2,Num+1);
Control_Point = zeros(Num+1,3);
phi = zeros(Num+1,1);
phi_1 = zeros(Num+1,1);
phi(1,1) = pi;
phi(end,1) = -pi;
m=1;
for u = umin : delta_u : umax
    % for u = 0 : 0.005 : 1
    for i = 0 : 1 : n-1
        Nik(i+1, 1) = BaseBezierFunction(i, k , u, NodeVector);
    end
    p_u = P * Nik;
    Dest(:,m) = p_u;
    
    %     line(p_u(1,1), p_u(2,1), 'Marker','.','LineStyle','-', 'Color','r','LineWidth',3);hold on;
    if m>1
        phi(m,1) = (Dest(2,m-1) - Dest(2,m))/(Dest(1,m-1) - Dest(1,m))-pi;
        %         phi_1(m,1) = (Dest(2,m-1) - Dest(2,m))/(Dest(1,m-1) - Dest(1,m));
    end
    m = m+1;
    %     Control_Point(m-1,:) = [Dest(:,m-1)' phi(m-1,1) phi_1(m-1,1)];
    Control_Point(m-1,:) = [Dest(:,m-1)' phi(m-1,1)];
end
Control_Point(m,:) = [goalPose(1:2) phi(m,1)];
CurrPose = StartPose;

Vehicle.WB = 3.7;  % [m] wheel base: rear to front steer
Vehicle.W = 2.6; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle 
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius

for n = 1:Num+1
    %     [poses,pathCosts,MotionLength] = RSheep(CurrPose,goalPose);
%     [isok,t,u,v] = LpSpRp(CurrPose(1),CurrPose(2),CurrPose(3)+pi,Rmin);
%     X = [isok,t,u,v];
%     disp(X);
    path = FindRSPath(CurrPose(1),CurrPose(2),CurrPose(3)+pi,Vehicle);
    PlotPath(path,Vehicle);
    Startslot = [StartPose(1) StartPose(2) StartPose(3)-pi];
    goalslot = [goalPose(1) goalPose(2) goalPose(3)+pi];
    PlotVehicle(Startslot(1),Startslot(2),Startslot(3));
    PlotVehicle(goalslot(1),goalslot(2),goalslot(3));
    plot(Control_Point(:,1),Control_Point(:,2),'Color','k','LineWidth',2);
    %     PlotVehicle(Control_Point(n,1),Control_Point(n,2),Control_Point(n,4));
    
    %     PlotVehicle(S,H,0);hold on;
    plot([-5 -1.5 -1.5 6 6 15],[1 1 -1.2 -1.2 1 1]);hold on;
    %     PlotVehicle(0,0,0);hold on;
    pause(0.1);
    hold off;
    goalPose = [randn(1)/Num randn(1)/Num randn(1)/Num].*3 + goalPose;
    CurrPose = Control_Point(n,:) + [randn(1)/Num randn(1)/Num randn(1)/Num].*3;
    Currslot = [CurrPose(1) CurrPose(2) CurrPose(3)+pi];
    PlotVehicle(Currslot(1),Currslot(2),Currslot(3));
end
PlotVehicle(Startslot(1),Startslot(2),Startslot(3));
PlotVehicle(goalslot(1),goalslot(2),goalslot(3));
plot(Control_Point(:,1),Control_Point(:,2),'Color','k','LineWidth',2);
