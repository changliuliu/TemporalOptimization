% Generates the figures in the paper
% Run temporal_opt.m and speed_iter.m first

switch scenario
    case 'Overtake'
        figure(1);clf;hold on
        subplot(511);hold on;
        yaxis = path(1,1:nstep);
        lanebd = 2;
        alongmax = 2.5;
        plot(path(1,[1,nstep]),[lanebd lanebd],'--k')
        plot(path(1,:),path(2,:),'k','LineWidth',2)
        axis([path(1,1) path(1,nstep) -2 6])
        title('Path')
        %ylabel('m')
        box on
        
        subplot(512);hold on;
        plot(yaxis,curv*100,'k','LineWidth',2)
        axis([path(1,1) path(1,nstep) -0.06*100 0.06*100])
        title('Curvature')
        box on
        subplot(513);hold on;
        for k=1:size(iter,2)
            plot(yaxis,[v0 iter{k}.speed],'color',[1-k/size(iter,2),1-k/size(iter,2),1-k/size(iter,2)],'LineWidth',1)
        end
        plot(yaxis,[v0 speed],'k','LineWidth',2)
        plot(yaxis([1,nstep]),[vref vref],'--k')
        axis([yaxis(1) yaxis(nstep) 7 12])
        title('The Speed Profile')
        %ylabel('m/s')
        box on
        
        subplot(514)
        hold on
        plot(yaxis(1:nstep-1),acc_long,'k','LineWidth',2)
        plot(yaxis(1:nstep-1),acc_lat,'-.k','LineWidth',2)
        plot(yaxis([1,nstep-1]),[alongmax alongmax],'--b')
        plot(yaxis([1,nstep-1]),-[alongmax alongmax],'--b')
        %legend('Longitudinal','Lateral','Limits')
        axis([yaxis(1) yaxis(nstep) -3 3 ])
        title('The Acceleration Profile')
        box on
        %ylabel('m/s^2')
        
        subplot(515)
        hold on
        plot(yaxis(1:nstep-1),j_long,'k','LineWidth',2)
        plot(yaxis(1:nstep-1),j_lat,'-.k','LineWidth',2)
        axis([yaxis(1) yaxis(nstep) -4 4 ])
        title('The Jerk Profile')
        %legend('Longitudinal','Lateral')
        %ylabel('m/s^3')
        
        
    case 'CurvRoad'
        figure(1);clf;hold on
        subplot(221);hold on;
        yaxis = 0:delta:(nstep-1)*delta;
        lanebd = 2;
        alongmax = 2.5;
        plot(yaxis,curv,'k','LineWidth',2)
        %plot(path(1,[1,nstep]),[lanebd lanebd],'--k')
        axis([yaxis(1) yaxis(end) 0 0.06])
        title('Curvature')
        %ylabel('1/m')
        box on
        subplot(223);hold on;
        for k=1:size(iter,2)
            plot(yaxis,[v0 iter{k}.speed],'color',[1-k/size(iter,2),1-k/size(iter,2),1-k/size(iter,2)],'LineWidth',1)
        end
        
        plot(yaxis,[v0 speed],'k','LineWidth',2)
        %plot(yaxis([1,nstep]),[vref vref],'k')
        
        %legend('Ref','Iter 1', 'Iter 2','Iter 3','Iter 4','Iter 5')
        axis([yaxis(1) yaxis(nstep) 7 12])
        title('The Speed Profile')
        %ylabel('m/s')
        %xlabel('Distance along the path (m)')
        box on
        
        subplot(222)
        hold on
        plot(yaxis(1:nstep-1),acc_long,'k','LineWidth',2)
        plot(yaxis(1:nstep-1),acc_lat,'-.k','LineWidth',2)
        plot(yaxis([1,nstep-1]),[alongmax alongmax],'--b')
        plot(yaxis([1,nstep-1]),-[alongmax alongmax],'--b')
        %legend('Longitudinal','Lateral','Limits')
        axis([yaxis(1) yaxis(nstep-1) -3 3 ])
        title('The Acceleration Profile')
        box on
        %ylabel('m/s^2')
        
        subplot(224)
        hold on
        plot(yaxis(1:nstep-1),j_long,'k','LineWidth',2)
        plot(yaxis(1:nstep-1),j_lat,'-.k','LineWidth',2)
        axis([yaxis(1) yaxis(nstep-1) -3 3 ])
        title('The Jerk Profile')
        %legend('Longitudinal','Lateral')
        %ylabel('m/s^3')
        
        
    case 'Intersec'
        figure(1);clf;hold on
        subplot(221);hold on;
        yaxis = 0:delta:(nstep-1)*delta;
        %yaxis = 0:nstep-1;
        plot(yaxis,curv,'k','LineWidth',2)
        %plot(path(1,[1,nstep]),[lanebd lanebd],'--k')
        axis([yaxis(1) yaxis(end) -0.2 0])
        title('Curvature')
        box on
        subplot(223);hold on;
        for k=1:size(iter,2)
            plot(yaxis,[v0 iter{k}.speed],'color',[1-k/size(iter,2),1-k/size(iter,2),1-k/size(iter,2)],'LineWidth',1)
        end
        plot(yaxis,[v0 speed],'k','LineWidth',2)
        plot(yaxis([1,nstep]),[vref vref],'k')
        axis([yaxis(1) yaxis(nstep) 0 6])
        title('The Speed Profile')
        %ylabel('m/s')
        box on
        
        subplot(222)
        hold on
        plot(yaxis(1:nstep-1),acc_long,'k','LineWidth',2)
        plot(yaxis(1:nstep-1),acc_lat,'-.k','LineWidth',2)
        plot(yaxis([1,nstep-1]),[alongmax alongmax],'--b')
        plot(yaxis([1,nstep-1]),-[alongmax alongmax],'--b')
        %legend('Longitudinal','Lateral','Limits')
        axis([yaxis(1) yaxis(nstep-1) -3 3 ])
        title('The Acceleration Profile')
        box on
        %ylabel('m/s^2')
        
        subplot(224)
        hold on
        plot(yaxis(1:nstep-1),j_long,'k','LineWidth',2)
        plot(yaxis(1:nstep-1),j_lat,'-.k','LineWidth',2)
        axis([yaxis(1) yaxis(nstep-1) -3 5 ])
        title('The Jerk Profile')
        %legend('Longitudinal','Lateral')
        %ylabel('m/s^3')
end


switch scenario
    case 'Overtake'
        xlabel('Distance along the lane (m)')
    case 'CurvRoad'
        %xlabel('Distance along the path (m)')
    case 'Intersec'
        %xlabel('Distance along the path (m)')
end

box on

%%
figure(2);hold on
%constraint_gen;
Accum = tril(ones(nstep,nstep-1),-1);

title('The Time Stamp')
switch scenario
    case 'Overtake'
        ylabel('Distance along the path (m)')
        plot(Accum*reft,0:delta:(nstep-1)*delta,'-ok','LineWidth',2,'MarkerSize',3);
    case 'CurvRoad'
        ylabel('Distance along the path (m)')
        plot(Accum*reft,yaxis,'-ok','LineWidth',2,'MarkerSize',3);
    case 'Intersec'
        ylabel('Distance along the path (m)')
        plot(Accum*reft,yaxis,'-ok','LineWidth',2,'MarkerSize',3);
end
xlabel('T(s)')
axis([0 8 yaxis(1) yaxis(end)+delta])
box on
%%
% figure(22);hold on
% plot(yaxis(1:nstep-1),reft,'-*','LineWidth',3);