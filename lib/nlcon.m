function [c,ceq] = nlcon(t,path,theta,v0,t0,alongmax,alatmax)
ceq = [];
nstep = length(theta)-1;
c = zeros(1,nstep*2);
f = zeros(2,1); h = zeros(2,1);
dp_long = zeros(1,nstep);
dp_lat = zeros(1,nstep);

for i=1:nstep
    for j=1:i%max([1,i-3]):i
        dp_long(j) = dot(path(:,j+1)-path(:,j),[cos(theta(i));sin(theta(i))]);
        dp_lat(j) = dot(path(:,j+1)-path(:,j),[sin(theta(i));-cos(theta(i))]);
    end
    if i > 1
        f(1) = 2*(t(i)*dp_long(i-1)-t(i-1)*dp_long(i));
        h(1) = t(i)*t(i-1)*(t(i)+t(i-1));
        f(2) = 2*(t(i)*dp_lat(i-1)-t(i-1)*dp_lat(i));
        h(2) = t(i)*t(i-1)*(t(i)+t(i-1));
    else
        f(1) = 2*(v0*t(1)-dp_long(1)); 
        h(1) = t(1)^2+t(1)*t0;
        f(2) = 2*(0*t(1)-dp_lat(1)); 
        h(2) = t(1)^2+t(1)*t0;
    end
    c(i) = abs(f(1)/h(1))-alongmax;
    c(i+nstep) = abs(f(2)/h(2))-alatmax;
end

end