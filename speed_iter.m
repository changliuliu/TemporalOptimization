% Show the speed profiles during the iterations
% Run temporal_opt.m first to load the scenario

orit = delta/vref*ones(nstep-1,1);
reft = orit;
iter = {};

for k = 1:100
% The constraint
[~,L,S,along,alat,jlong,jlat,speed] = t2y(reft,path,theta,vref,[v0;0],delta/v0);
iter{k}.t = reft;
iter{k}.along = along;
iter{k}.alat = alat;
iter{k}.jlong = jlong;
iter{k}.jlat = jlat;
iter{k}.speed = speed;
Lstack = [L;A];
Sstack = [S;b];
% QP
soln = quadprog(Q,[],Lstack,Sstack);

% [~,Aeq,beq] = t2y_lin(reft,path,theta,vref,[v0;0],delta/v0);
% soln = quadprog(Q,[],A,b,Aeq,beq);
reftnew = soln(1:nstep-1);

if norm(reftnew-reft) < 0.0001
    disp(['converged at step ',num2str(k)]);
    break
end
reft = reftnew;


end
k = k+1;
[~,~,~,along,alat,jlong,jlat,speed] = t2y(reft,path,theta,vref,[v0;0],delta/v0);
iter{k}.t = reft;
iter{k}.along = along;
iter{k}.alat = alat;
iter{k}.jlong = jlong;
iter{k}.jlat = jlat;
iter{k}.speed = speed;

%%
figure(3); hold on
for k=1:size(iter,2)
plot(path(1,1:nstep),[v0 iter{k}.speed],'color',[1-k/size(iter,2),1-k/size(iter,2),1-k/size(iter,2)],'LineWidth',2)
%pause
end
box on
axis([path(1,1) path(1,nstep) 0 12 ])
title('The Speed Profile Under different Iterations')
ylabel('m/s')
xlabel('Distance along the lane (m)')
box on
legend('Ref','Iter 1', 'Iter 2','Iter 3','Iter 4','Iter 5')
%%
figure(4); hold on
constraint_gen(path,scenario);
Accum = tril(ones(nstep,nstep-1),-1);
for k=1:size(iter,2)
plot(Accum*iter{k}.t,path(1,1:nstep),'-*','color',[1-k/size(iter,2),1-k/size(iter,2),1-k/size(iter,2)]);
end
title('The Time Stamp')
ylabel('m')
xlabel('T(s)')
box on
%%
figure(5); hold on
for k=1:size(iter,2)
plot(path(1,1:nstep-1),iter{k}.along,'color',[1-k/size(iter,2),1-k/size(iter,2),1-k/size(iter,2)])
%pause(1)
end

