
% path = FindRSPath(1,1,pi);
path = FindRSPath(5,0,0);
PlotPath(path);

%% 函数

% formula 8.1
function [isok,t,u,v] = LpSpLp(x,y,phi)
    [t,u] = cart2pol(x-sin(phi),y-1+cos(phi));
    if t >= 0
        v = mod2pi(phi-t);
        if v >= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

function v = mod2pi(x)
    v = rem(x,2*pi);
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end

function [tau,omega] = tauOmega(u,v,xi,eta,phi)
    delta = mod2pi(u-v);
    A = sin(u)-sin(delta);
    B = cos(u)-cos(delta)-1;
    t1 = atan2(eta*A-xi*B,xi*A+eta*B);
    t2 = 2*(cos(delta)-cos(v)-cos(u))+3;
    if t2 < 0
        tau = mod2pi(t1+pi);
    else
        tau = mod2pi(t1);
    end
    omega = mod2pi(tau-u+v-phi);
end

% formula 8.2
function [isok,t,u,v] = LpSpRp(x,y,phi)
    [t1,u1] = cart2pol(x+sin(phi),y-1-cos(phi));
    if u1^2 >= 4
        u = sqrt(u1^2-4);
        theta = atan2(2,u);
        t = mod2pi(t1+theta);
        v = mod2pi(t-phi);
        if t >= 0 && v >= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.3/8.4
function [isok,t,u,v] = LpRmL(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,u1] = cart2pol(xi,eta);
    if u1 <= 4
        u = -2*asin(u1/4);
        t = mod2pi(theta+u/2+pi);
        v = mod2pi(phi-t+u);
        if t >= 0 && u <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.7
function [isok,t,u,v] = LpRupLumRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (2+sqrt(xi^2+eta^2))/4;
    if rho <= 1
        u = acos(rho);
        [t,v] = tauOmega(u,-u,xi,eta,phi);
        if t >= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.8
function [isok,t,u,v] = LpRumLumRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (20-xi^2-eta^2)/16;
    if rho >= 0 && rho <= 1
        u = -acos(rho);
        if u >= -pi/2
            [t,v] = tauOmega(u,u,xi,eta,phi);
            if t >=0 && v >=0
                isok = true;
                return
            end
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.9
function [isok,t,u,v] = LpRmSmLm(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,rho] = cart2pol(xi,eta);
    if rho >= 2
        r = sqrt(rho^2-4);
        u = 2-r;
        t = mod2pi(theta+atan2(r,-2));
        v = mod2pi(phi-pi/2-t);
        if t >= 0 && u <= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.10
function [isok,t,u,v] = LpRmSmRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [theta,rho] = cart2pol(-eta,xi);
    if rho >= 2
        t = theta;
        u = 2-rho;
        v = mod2pi(t+pi/2-phi);
        if t >= 0 && u <= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.11
function [isok,t,u,v] = LpRmSLmRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [~,rho] = cart2pol(xi,eta);
    if rho >= 2
        u = 4-sqrt(rho^2-4);
        if u <= 0
            t = mod2pi(atan2((4-u)*xi-2*eta,-2*xi+(u-4)*eta));
            v = mod2pi(t-phi);
            if t >= 0 && v >= 0
                isok = true;
                return
            end
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

function [isok,path] = CSC(x,y,phi)
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpSpLp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(15,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(15,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(16,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,-y,phi); % timeflp + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(16,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(13,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(13,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(14,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(14,:),-t,-u,-v,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCC(x,y,phi)
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmL(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),-t,-u,-v,0,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [isok,t,u,v] = LpRmL(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),-v,-u,-t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),-v,-u,-t,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCCC(x,y,phi)
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRupLumRm(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),-t,-u,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),-t,-u,-u,-v,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCSC(x,y,phi)
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmSmLm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(5,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(5,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(6,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(6,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(9,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(9,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(10,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(10,:),-t,pi/2,-u,-v,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [isok,t,u,v] = LpRmSmLm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(7,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(7,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(8,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(8,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(11,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(11,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(12,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(12,:),-v,-u,pi/2,-t,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCSCC(x,y,phi)
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmSLmRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(17,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(17,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(18,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(18,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end


function path = FindRSPath(x,y,phi)
    rmin = 5; %minimum turning radius
    x = x/rmin;
    y = y/rmin;
    [isok1,path1] = CSC(x,y,phi);
    [isok2,path2] = CCC(x,y,phi);
    [isok3,path3] = CCCC(x,y,phi);
    [isok4,path4] = CCSC(x,y,phi);
    [isok5,path5] = CCSCC(x,y,phi);
    isoks = [isok1, isok2, isok3, isok4, isok5];
    paths = {path1, path2, path3, path4, path5};
    Lmin = inf;
    for i = 1:5
        if isoks(i) == true
            elem = paths{i};
            if Lmin > elem.totalLength
                Lmin = elem.totalLength;
                path = elem;
            end
        end
    end
end

function PlotPath(path)
    type = path.type;
%     x = [];
%     y = [];
    seg = [path.t,path.u,path.v,path.w,path.x];
    pvec = [0,0,0];
    rmin = 5;
    for i = 1:5        
        if type(i) == RSPathElem.RS_STRAIGHT
            theta = pvec(3);
            dl = rmin*seg(i);
            dvec = [dl*cos(theta), dl*sin(theta), 0];
            dx = pvec(1)+linspace(0,dvec(1));
            dy = pvec(2)+linspace(0,dvec(2));
%             x = [x,dx];
%             y = [y,dy];
            pvec = pvec+dvec;
        elseif type(i) == RSPathElem.RS_LEFT
            theta = pvec(3);
            dtheta = seg(i);
            cenx = pvec(1)-rmin*sin(theta);
            ceny = pvec(2)+rmin*cos(theta);
            t = theta-pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
%             x = [x,dx];
%             y = [y,dy];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = dtheta;
        elseif type(i) == RSPathElem.RS_RIGHT
            theta = pvec(3);
            dtheta = -seg(i);
            cenx = pvec(1)+rmin*sin(theta);
            ceny = pvec(2)-rmin*cos(theta);
            t = theta+pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
%             x = [x,dx];
%             y = [y,dy];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = -dtheta;
        else
            % do nothing
        end
        if dl > 0
            plot(dx,dy,'b');
        else
            plot(dx,dy,'r');
        end
        hold on
    end
    hold off
    axis equal
end

