function [isok,path] = CCSCC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmSLmRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(17,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(17,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(18,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(18,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end