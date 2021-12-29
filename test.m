% path = FindRSPath(5,1,pi);
clc;clear;close all;
Vehicle.WB = 3.7;  % [m] wheel base: rear to front steer
Vehicle.W = 2.6; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius
x0 = 9;y0 = 3; phi0  = 0;
v0 = 1 ;N = 500; Ts = 0.2;
for xx = 9:-1:4
    for yy = 3:-0.3:2
        for phii = 0:0.3:0.9
            path = FindRSPath(xx,yy,phii,Vehicle);
            n = 15; k = 5; % 设计样条曲线% n,线条控制点数；k，阶次数
            x = [];y = [];angle = [];
            type = path.type;
            seg = [path.t,path.u,path.v,path.w,path.x];
            pvec = [0,0,0];rmin = Vehicle.MIN_CIRCLE;
            for i = 1:5
                if type(i) == 'S'
                    theta = pvec(3); %上一轨迹段终点航向；
                    dl = rmin*seg(i); %seg(i)为归一化后直线段长度，还原到归一化前轨迹直线长度 * rmin
                    dvec = [dl*cos(theta), dl*sin(theta), 0]; % 直线段起始点坐标差；
                    dx = pvec(1)+linspace(0,dvec(1)); % 在直线轨迹上离散化100个点
                    dy = pvec(2)+linspace(0,dvec(2));
                    x = [x,dx];
                    y = [y,dy];
                    pvec = pvec+dvec;
                elseif type(i) == 'L'
                    theta = pvec(3); %上一轨迹段终点航向；
                    dtheta = seg(i);
                    cenx = pvec(1)-rmin*sin(theta); % [-pi,pi),极坐标，四象限都成立；
                    ceny = pvec(2)+rmin*cos(theta); % 以rmin计算L左向前起始圆弧圆心坐标；
                    t = theta-pi/2+linspace(0,dtheta); %以起始坐标theta为参考，自动生成等间距100个点；dtheta>0为沿起始点圆弧 逆时针，前进；dtheta<0,顺时针，倒退；
                    dx = cenx+rmin*cos(t); % 将极坐标变换到与theta顺时针旋转90度为起始点；朝下
                    dy = ceny+rmin*sin(t);
                    x = [x,dx];
                    y = [y,dy];
                    angle=[angle,t]; %极坐标参考下的圆弧对应角度；
                    theta = theta+dtheta; %轨迹结束时的航向；
                    pvec = [dx(end),dy(end),theta];  %轨迹结束时的坐标点
                    dl = dtheta; % 轨迹段行驶的角度，t,u,v,w,小于0时为倒退，大于0时为前进；
                elseif type(i) == 'R'
                    theta = pvec(3);
                    dtheta = -seg(i); %seg(i)<0,为倒退，画轨迹时，dtheta应为正，逆时针；
                    cenx = pvec(1)+rmin*sin(theta); % [-pi,pi),极坐标，四象限都成立；
                    ceny = pvec(2)-rmin*cos(theta); % 以rmin计算R右向前起始圆弧圆心坐标；
                    t = theta+pi/2+linspace(0,dtheta); %以起始坐标theta为参考，自动生成等间距100个点；dtheta>0为沿起始点圆弧 逆时针，倒退；dtheta<0,顺时针，前进；
                    dx = cenx+rmin*cos(t); % 将极坐标变换到与theta逆时针旋转90度为起始点；朝上
                    dy = ceny+rmin*sin(t);
                    x = [x,dx];
                    y = [y,dy];
                    angle=[angle,t]; %极坐标参考下的圆弧对应角度；
                    theta = theta+dtheta; %轨迹结束时的航向；
                    pvec = [dx(end),dy(end),theta]; %轨迹结束时的坐标点
                    dl = -dtheta; % 轨迹段行驶的角度，t,u,v,w,小于0时为倒退，大于0时为前进；
                else
                    % do nothing
                end
                %                     if dl > 0
                %                         plot(dx,dy,'b','LineWidth',2);
                %                     else
                %                         plot(dx,dy,'r','LineWidth',2);
                %                     end
                %                     hold on
            end
            %     p = zeros(n,2); % 取RS路径上n个点
            %     ds = floor(299/(n-1))+1;
            %     p(:,1) = [x(1:ds:300)';x(end)];
            %     p(:,2) = [y(1:ds:300)';y(end)];
            %     P_dest = p(1:n,1:2)';
            % %     NodeVector = linspace(0,1,n+k+1); % 从0到1，设置均匀B样条节点区间；
            %     NodeVector = [0,0,0,0,0,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1,1,1,1,1];
            %     Nik = zeros(n+k, k);
            %     plot(P_dest(1, 1:n), P_dest(2, 1:n),...
            %         'o','LineWidth',1,...
            %         'MarkerEdgeColor','k',...
            %         'MarkerFaceColor','g',...
            %         'MarkerSize',6);hold on;
            %     line(P_dest(1, 1:n), P_dest(2, 1:n));hold on;
            %     umin =0;umax =1;delta_u = 0.01;
            %     Num = floor((umax-umin)/delta_u)+1;
            %     Dest = zeros(2,Num);
            %     m=1;
            %     N_i_5 = zeros(n,1);
            %     for u = umin : delta_u : umax
            %         for ii = 1:n
            %             N_i_5(ii,1) = BaseBezierFunction(ii-1, k , u, NodeVector);
            %         end
            %         %     N_i_5 =
            %         p_u = P_dest * N_i_5;
            %         Dest(:,m) = p_u;
            %         m = m+1;
            %         line(p_u(1,1), p_u(2,1),'LineWidth',3, 'Marker','.','LineStyle','-', 'Color','r');hold on;
            %     end
            PlotPath(path,Vehicle);
            pause(0.2);
            hold on;
        end
    end
end

