function [path,theta,curv] = path_gen(scenario)
switch scenario
    case 'CurvRoad'
        %% Generate Reference Path for overtake based on a 6-th order polynomial
        s = 50;
        delta = 2;
        maxcurv = 0.05;
        lane_width = 4;
        tail = 20;
        
        M = [s^2 s 1;5*s^2 4*s 3;20*s^2 12*s 6];
        V = [1;6;30]*s^3;
        vec = -inv(M)*V;
        x = s/2;
        factor = x^6+[x^5 x^4 x^3]*vec;
        
        path = zeros(2,(s+tail)/delta);
        theta = zeros(1,(s+tail)/delta);
        curv = zeros(1,(s+tail)/delta);
        for i = 1:(s+tail)/delta
            path(1,i) = (i-1)*delta;
            x = i*delta - floor(tail/2);
            if x >= 0 && x <= s
                curv(i) = (x^6+[x^5 x^4 x^3]*vec)/factor*maxcurv;
            end
        end
        theta = cumtrapz(curv)*delta;
        path = [cumtrapz(cos(theta));cumtrapz(sin(theta))]*delta;
    case 'Overtake'
        %% Generate Reference Path for overtake based on a 6-th order polynomial
        s = 60;
        delta = 2;
        lane_width = 4;
        tail = 8;
        maxcurv = 0.04;
        M = [s^2 s 1;5*s^2 4*s 3;20*s^2 12*s 6];
        V = [1;6;30]*s^3;
        vec = -inv(M)*V;
        x = s/2;
        factor = x^6+[x^5 x^4 x^3]*vec;
        n = 5;
        path = zeros(2,(s+tail)/delta);
        theta = zeros(1,(s+tail)/delta);
        curv = zeros(1,(s+tail)/delta);
        for i = 1:(s+tail)/delta
            path(1,i) = (i-1)*delta;
            x = i*delta - floor(tail/2);
            if x >= 0 && x <= s/n*(n-1)/2
                curv(i) = sin(x/(s/n*(n-1))*4*pi)*maxcurv;
            end
            if x >= s/n*(n+1)/2 && x <= s
                curv(i) = sin((s-x)/(s/n*(n-1))*4*pi)*maxcurv;
            end
        end
        for iter = 1:4
            newcurv = curv;
            for i = 2:(s+tail)/delta-1
                newcurv(i) = curv(i)/2+(curv(i-1)+curv(i+1))/4;
            end
            newcurv(1) = curv(1)/2 + curv(2)/4;
            newcurv(end) = curv(end)/2 +curv(end-1)/4;
            curv = newcurv;
        end
        theta = cumtrapz(curv)*delta;
        path = [cumtrapz(cos(theta));cumtrapz(sin(theta))]*delta;

    case 'Intersec'
        %% Reference path for turn
        %% Generate Reference Path for overtake based on a 6-th order polynomial
        s = 18;
        delta = 0.5;
        lane_width = 4;
        tail = 0;
        
        M = [s^2 s 1;5*s^2 4*s 3;20*s^2 12*s 6];
        V = [1;6;30]*s^3;
        vec = -inv(M)*V;
        x = s/2;
        factor = x^6+[x^5 x^4 x^3]*vec;
        
        path = zeros(2,(s+tail)/delta);
        theta = zeros(1,(s+tail)/delta);
        curv = zeros(1,(s+tail)/delta);
        for i = 1:(s+tail)/delta
            path(1,i) = (i-1)*delta;
            x = i*delta - floor(tail/2);
            if x >= 0 && x <= s
                curv(i) = -(x^6+[x^5 x^4 x^3]*vec)/factor*0.2;
            end
        end
        theta = cumtrapz(curv)*delta+pi/2;
        path = [cumtrapz(cos(theta));cumtrapz(sin(theta))]*delta;
        for i=1:size(path,2)
            path(:,i)=path(:,i)+[2;-11];
        end
end
%% Plot Path
figure(1);hold on
plot(path(1,:),path(2,:),'-*')
plot(path(1,:),theta(:));
legend('path','theta')
axis equal


