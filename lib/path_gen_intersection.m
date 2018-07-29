%% Generate map
lane_width = 4;
length = 10;
ped = 1.5;
figure(1);clf;hold on
plot([lane_width,length],[0,0],'--k')
plot(-[lane_width,length],[0,0],'--k')
plot([0,0],[lane_width,length],'--k')
plot([0,0],-[lane_width,length],'--k')
plot([lane_width, lane_width]+ped,[0 lane_width],'k')
plot(-[lane_width, lane_width]-ped,[0 -lane_width],'k')
plot([0 -lane_width],[lane_width, lane_width]+ped,'k')
plot([0 lane_width],-[lane_width, lane_width]-ped,'k')
plot([lane_width lane_width length],[-length -lane_width -lane_width],'k','LineWidth',5)
plot([lane_width lane_width length],-[-length -lane_width -lane_width],'k','LineWidth',5)
plot([-length -lane_width -lane_width],-[lane_width lane_width length],'k','LineWidth',5)
plot([-length -lane_width -lane_width],[lane_width lane_width length],'k','LineWidth',5)
axis equal
axis([-10 10 -10 10])
box on
%% Reference path for turn

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
% r = 5;
% n = 4;
% s = 12;
% delta = 0.5;
% path = zeros(2,s/delta);
% crit = -r-lane_width/2;
% str_length = round((s-pi*r/2)/3*2);
% start = [lane_width/2; (crit-str_length)+delta];
% path(:,1) = start;
% theta = zeros(1,s/delta);
% theta(1) = pi/2;
% for i = 2:str_length/delta
%     path(:,i) = path(:,i-1) + [0;delta];
%     theta(i) = pi/2;
% end
% ksp = ceil(pi*r/2)-1;
% dtheta = pi/2/ksp*delta;
% for i = str_length/delta+1 : str_length/delta + ksp/delta
%     path(:,i) = [-crit;crit] + r*[cos(pi-dtheta*(i-str_length/delta));sin(pi-dtheta*(i-str_length/delta))];
%     theta(i) = atan2(-cos(pi-dtheta*(i-str_length/delta)),sin(pi-dtheta*(i-str_length/delta)));
% end
% start = [-crit;-lane_width/2];
% for i = str_length/delta+ksp/delta + 1: s/delta
%     path(:,i) = start + [delta;0]*(i-str_length/delta-ksp/delta);
%     theta(i) = 0;
% end
plot(path(1,:),path(2,:),'k','LineWidth',3)
% plot(theta)
