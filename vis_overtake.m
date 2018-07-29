% Visualize the trajectory during overtake
% Run temporal_opt.m with "Overtake" option to get the trajectory first

figure;hold on
plot(path(1,[1,nstep]),[lanebd lanebd],'--k')
plot(path(1,:),path(2,:),'color',[0.9,0.9,0.9],'LineWidth',1)
Tstamp = Accum*reft;
% Vehicle 1
v1 = 6; s1 = 9;
% Vehicle 2
v2 = -11; s2 = 114;
width = 4;
axis equal
axis([path(1,1) path(1,nstep) -lanebd lanebd*3 ])
box on
for i = 1:1:nstep
    if i > 1
        delete([handle.ego.edge,handle.ego.fill]);
        delete([handle.v1.edge,handle.v1.fill]);
        delete([handle.v2.edge,handle.v2.fill]);
    end
    handle.ego = show_vehicle(path(:,i),theta(i),[0.7,0.7,0.7]);
    handle.v1 = show_vehicle([v1*Tstamp(i)+s1;0],0,'y');
    handle.v2 = show_vehicle([v2*Tstamp(i)+s2;width],0,'y');
    title(['s = ',num2str((i-1)*delta)])
    pause
end
axis equal
axis([path(1,1) path(1,nstep) -lanebd lanebd*3 ])