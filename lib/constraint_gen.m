function [A,b] = constraint_gen(path,scenario)

nstep = size(path,2);

switch scenario
    case 'Overtake'
        %% Generate constraint
        % Vehicle 1
        v1 = 6; s1 = 9;
        % Vehicle 2
        v2 = -11; s2 = 116;
        
        safe_margin = 5;
        % Acceleration bound
        alongmax = 2.5;
        alatmax = 2.5;
        
        M = eye(5*(nstep-1));
        A = [tril(ones(nstep-1,6*(nstep-1)),0);
            -tril(ones(nstep-1,6*(nstep-1)),0);
            zeros(nstep-1) M(1:5:end,:);
            zeros(nstep-1) M(2:5:end,:)];
        b = [zeros(2*(nstep-1),1);
            alongmax*ones(nstep-1,1);
            alatmax*ones(nstep-1,1);];
        lanebd = 2;
        margin = 0.5;
        overtake = 0;
        figure(2);hold on;
        for i=2:nstep
            point = path(:,i);
            t_min = 0; t_max = 10;
            % Consider vehicle 2
            if point(2) > lanebd - margin
                t_max = min([t_max,(point(1)-s2+safe_margin)/v2]);
            end
            % Consider vehicle 1
            if point(2) < lanebd + margin
                if overtake
                    t_max = min([t_max,(point(1)-s1-safe_margin)/v1]);
                else
                    t_min = max([t_min,(point(1)-s1+safe_margin)/v1]);
                end
            else
                overtake = 1;
            end
            b(i-1) = t_max;
            b(i-1+nstep-1) = -t_min;
            plot([t_min,t_max],[i-1 i-1]*2)
        end
    case 'CurvRoad'
        % Acceleration bound
        alongmax = 2.5;
        alatmax = 2.5;
        
        M = eye(5*(nstep-1));
%         A = [zeros(nstep-1) M(1:5:end,:);
%             zeros(nstep-1) M(2:5:end,:)];
%         b = [alongmax*ones(nstep-1,1);
%             alatmax*ones(nstep-1,1);];
        A = [zeros(nstep-1) M(1:5:end,:);
            zeros(nstep-1) M(2:5:end,:);
            zeros(nstep-1) -M(1:5:end,:);
            zeros(nstep-1) -M(2:5:end,:)];
        b = [alongmax*ones(nstep-1,1);
            alatmax*ones(nstep-1,1);
            alongmax*ones(nstep-1,1);
            alatmax*ones(nstep-1,1);];
    case 'Intersec'
        %% Generate constraint
        % Vehicle 1
        v1 = 6; s1 = 9;
        % Vehicle 2
        v2 = -11; s2 = 116;
        
        safe_margin = 5;
        % Acceleration bound
        alongmax = 2.5;
        alatmax = 2.5;
        
        M = eye(5*(nstep-1));
        A = [tril(ones(nstep-1,6*(nstep-1)),0);
            -tril(ones(nstep-1,6*(nstep-1)),0);
            zeros(nstep-1) M(1:5:end,:);
            zeros(nstep-1) M(2:5:end,:)];
        b = [zeros(2*(nstep-1),1);
            alongmax*ones(nstep-1,1);
            alatmax*ones(nstep-1,1);];
        figure(2);hold on;
        for i=2:nstep
            point = path(:,i);
            t_min = 0; t_max = 15;
            if point(1)>3.6 && point(1)<7.5
                t_min = 5;
                t_max = 6;
            end

            b(i-1) = t_max;
            b(i-1+nstep-1) = -t_min;
            plot([t_min,t_max],[i-1 i-1]*0.5,'k')
        end
end