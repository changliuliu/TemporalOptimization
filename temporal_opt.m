%% Code for Temporal Optimization
% For IVS 17 paper "Speed Profile Generation in Dynamic Environments via Temporal Optimization"
%
% Changliu Liu
% 2017.1

addpath('lib')
%% Specify Scenario and Solver
scenario = 'Overtake'; % Overtake, Intersec, CurvRoad
SOLVER = 1; % 1:CFS, 2:SQP, 3:TWO

%% Parameters
switch scenario
    case 'Overtake'
        delta = 2;
        v0 = 10;
        vref = 11;
    case 'CurvRoad'
        delta = 2;
        v0 = 11;
        vref = 11;
    case 'Intersec'
        delta = 0.5;
        v0 = 2.5;
        vref = 5;
end
%% Generate Path
[path,theta,curv] = path_gen(scenario);
nstep = size(path,2);
refpath = [];
for i=1:nstep
    refpath = [refpath;path(:,i)];
end
%% Generate Constraints
[A,b] = constraint_gen(path,scenario);
switch scenario
    case 'CurvRoad'
        orit = delta/v0*ones(nstep-1,1);
    case 'Overtake'
        orit = delta/v0*ones(nstep-1,1);
    case 'Intersec'
        orit = delta/vref*ones(nstep-1,1);
end
reft = orit;
dim = 5;
%% The cost function
switch scenario
    case 'CurvRoad'
        Q = diag([zeros(nstep-1,1);repmat([1;10;10;10;10],nstep-1,1)]);
    case 'Overtake'
        Q = diag([zeros(nstep-1,1);repmat([1;10;10;10;10],nstep-1,1)]);
    case 'Intersec'
        Q = diag([zeros(nstep-1,1);repmat([1;10;10;10;10],nstep-1,1)]);
        %Q(end,end) = 100;
end

fun = @(t) temporal_cost(t,path,theta,vref,Q,[v0;0],delta/v0);
figure(1);
hold on
plot(path(1,:),path(2,:))
if min(scenario == 'Overtake') || min(scenario == 'CurvRoad')
    %yaxis = path(1,:);
    yaxis = 0:delta:(nstep-1)*delta
else
    yaxis = 0:delta:(nstep-1)*delta;
end

%% Solve the problem
%% CFS
if SOLVER == 1 || SOLVER == 3
    tic
    for k = 1:1000
        FEAS = 1;
        [~,L,S] = t2y(reft,path,theta,vref,[v0;0],delta/v0);
        Lstack = [L;A];
        Sstack = [S;b];
        soln = quadprog(Q,[],Lstack,Sstack);

%         [~,Aeq,beq] = t2y_lin(reft,path,theta,vref,[v0;0],delta/v0);
%         soln = quadprog(Q,[],A,b,Aeq,beq);
%         soln_lin = quadprog(Q,[],A,b,L(1:5*(nstep-1),:),S(1:5*(nstep-1)));
%         soln_lin = quadprog(Q,[],A,b,L(5*(nstep-1)+1:end,:),S(5*(nstep-1)+1:end));
        
        reftnew = soln(1:nstep-1);
        
        if norm(reftnew-reft) < 0.0001
            disp(['converged at step ',num2str(k)]);
            break
        end
        reft = reftnew;
    end
    time = toc
    disp('final cost: ');
    %fun(reft)
    [~,~,~,acc_long,acc_lat,j_long,j_lat,speed] = t2y(reft,path,theta,vref,[v0;0],delta/v0);
    plot(yaxis(1:nstep),[v0 speed])
    plot(yaxis(1:nstep-1),acc_long)
    plot(yaxis(1:nstep-1),acc_lat)
    plot(yaxis(1:nstep-1),j_long,'--')
    plot(yaxis(1:nstep-1),j_lat,'--')
end
%% SQP
if SOLVER == 2 || SOLVER == 3
    options = optimoptions(@fmincon,'Display','Iter','Algorithm','sqp','TolX',0.00001,'MaxFunEvals',30000);
    nl = @(t) nlcon(t,path,theta,vref,delta/vref,2.5,2.5);
    tic
    [t_sqp,fval,exitflag,output] = fmincon(fun,orit,A(:,1:nstep-1),b,[],[],[],[],nl,options);
    sqptime = toc
    [~,~,~,acc_long_sqp,acc_lat_sqp,j_long_sqp,j_lat_sqp,speed_sqp] = t2y(t_sqp,path,theta,vref);
    plot(yaxis(1:nstep),[v0 speed_sqp])
    plot(yaxis(1:nstep-1),acc_long_sqp)
    plot(yaxis(1:nstep-1),acc_lat_sqp)
    plot(yaxis(1:nstep-1),j_long_sqp,'--')
    plot(yaxis(1:nstep-1),j_lat_sqp,'--')
end
%% Legend
if SOLVER == 1
    legend('path','speed profile','longitudial accleration','lateral acceleration','longitudial jerk','lateral jerk')
end
if SOLVER == 2
    legend('path','speed profile sqp','longitudial accleration sqp','lateral acceleration sqp','longitudial jerk sqp','lateral jerk sqp')
end
if SOLVER == 3
    legend('path','speed profile','longitudial accleration','lateral acceleration','longitudial jerk','lateral jerk',...
        'speed profile sqp','longitudial accleration sqp','lateral acceleration sqp','longitudial jerk sqp','lateral jerk sqp')
end
%% Plot time stamps
Accum = tril(ones(nstep,nstep-1),-1);
figure(2);hold on

if SOLVER == 1 || SOLVER == 3
    plot(Accum*reft,yaxis,'-*');
end
if SOLVER == 2 || SOLVER == 3
    plot(Accum*t_sqp,yaxis,'-.');
end
if SOLVER == 3
    legend('SCFS','SQP');
end