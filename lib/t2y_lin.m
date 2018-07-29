function [refy,A,b,varargout] = t2y_lin(t,path,theta,vref,varargin)
if ~isempty(varargin)
    v0 = varargin{1};
    t0 = varargin{2};
else
    v0 = [vref;0];
    t0 = 2/vref;
end
nstep = size(path,2)-1;

dp_long = zeros(1,nstep);
dp_lat = zeros(1,nstep);

refy = zeros(nstep*5,1);
A = zeros(5*nstep,nstep*6);
b = zeros(5*nstep,1);
along = zeros(1,nstep);
alat = zeros(1,nstep);
jlong = zeros(1,nstep);
jlat = zeros(1,nstep);
vlong = zeros(1,nstep);
for i=1:nstep
    for j=max([1,i-3]):i
        dp_long(j) = dot(path(:,j+1)-path(:,j),[cos(theta(i));sin(theta(i))]);
        dp_lat(j) = dot(path(:,j+1)-path(:,j),[sin(theta(i));-cos(theta(i))]);
    end
    f = zeros(5,1);
    h = zeros(5,1);
    df = zeros(5,nstep);
    dh = zeros(5,nstep);
    if i > 1
        f(1) = 2*(t(i)*dp_long(i-1)-t(i-1)*dp_long(i));
        h(1) = t(i)*t(i-1)*(t(i)+t(i-1));
        f(2) = 2*(t(i)*dp_lat(i-1)-t(i-1)*dp_lat(i));
        h(2) = t(i)*t(i-1)*(t(i)+t(i-1));
        
        df(1,i) = 2*dp_long(i-1);
        df(1,i-1) = -2*dp_long(i);
        dh(1,i) = 2*t(i)*t(i-1)+t(i-1)^2;
        dh(1,i-1) = 2*t(i)*t(i-1)+t(i)^2;
        
        df(2,i) = 2*dp_lat(i-1);
        df(2,i-1) = -2*dp_lat(i);
        dh(2,i) = 2*t(i)*t(i-1)+t(i-1)^2;
        dh(2,i-1) = 2*t(i)*t(i-1)+t(i)^2;
    else
        f(1) = 2*(v0(1)*t(1)-dp_long(1)); 
        h(1) = t(1)^2+t(1)*t0;
        f(2) = 2*(v0(2)*t(1)-dp_lat(1)); 
        h(2) = t(1)^2+t(1)*t0;
        
        df(1,1) = 2*v0(1);
        dh(1,1) = 2*t(1)+t0;
        
        df(2,1) = 2*v0(2);
        dh(2,1) = 2*t(1)+t0;
    end
    if i > 2
        f(3) = 6*(t(i-1)*t(i-2)*(t(i-1)+t(i-2))*dp_long(i) - (t(i)+2*t(i-1)+t(i-2))*t(i)*t(i-2)*dp_long(i-1) ...
            +t(i)*t(i-1)*(t(i)+t(i-1))*dp_long(i-2));
        h(3) = t(i)*t(i-1)*t(i-2)*(t(i)+t(i-1))*(t(i)+t(i-1)+t(i-2));
        f(4) = 6*(t(i-1)*t(i-2)*(t(i-1)+t(i-2))*dp_lat(i) - (t(i)+2*t(i-1)+t(i-2))*t(i)*t(i-2)*dp_lat(i-1) ...
            +t(i)*t(i-1)*(t(i)+t(i-1))*dp_lat(i-2));
        h(4) = t(i)*t(i-1)*t(i-2)*(t(i)+t(i-1))*(t(i)+t(i-1)+t(i-2));
        
        df(3,i) = -6*(2*t(i)+2*t(i-1)+t(i-2))*t(i-2)*dp_long(i-1) ...
                +6*(2*t(i)+t(i-1))*t(i-1)*dp_long(i-2);
        df(3,i-1) = 6*(2*t(i-1)+t(i-2))*t(i-2)*dp_long(i)...
                -6*2*t(i)*t(i-2)*dp_long(i-1)...
                +6*(t(i)+2*t(i-1))*t(i)*dp_long(i-2);
        df(3,i-2) = -6*(2*t(i-2)+2*t(i-1)+t(i))*t(i)*dp_long(i-1) ...
                +6*(2*t(i-2)+t(i-1))*t(i-1)*dp_long(i);
        dh(3,i) = t(i-1)*t(i-2)*(t(i)^2+t(i-1)*t(i)+(2*t(i)+t(i-1))*(t(i)+t(i-1)+t(i-2)));
        dh(3,i-1) = t(i)*t(i-2)*(t(i-1)^2+t(i-1)*t(i)+(2*t(i-1)+t(i))*(t(i)+t(i-1)+t(i-2)));
        dh(3,i-2) = t(i)*t(i-1)*(t(i)+t(i-1))*(2*t(i-2)+t(i)+t(i-1));
        
        df(4,i) = -6*(2*t(i)+2*t(i-1)+t(i-2))*t(i-2)*dp_lat(i-1) ...
                +6*(2*t(i)+t(i-1))*t(i-1)*dp_lat(i-2);
        df(4,i-1) = 6*(2*t(i-1)+t(i-2))*t(i-2)*dp_lat(i)...
                -6*2*t(i)*t(i-2)*dp_lat(i-1)...
                +6*(t(i)+2*t(i-1))*t(i)*dp_lat(i-2);
        df(4,i-2) = -6*(2*t(i-2)+2*t(i-1)+t(i))*t(i)*dp_lat(i-1) ...
                +6*(2*t(i-2)+t(i-1))*t(i-1)*dp_lat(i);
        dh(4,:) = dh(3,:);
        
    else
        
            f(3) = 0;
            h(3) = 1;
            f(4) = 0;
            h(4) = 1;
        if i == 1
            
            f(3) = 6*(v0(1)*t(1)-dp_long(1));
            h(3) = (t(1)+2*t0)*(t(1)^2+t(1)*t0);
            f(4) = 6*(v0(2)*t(1)-dp_lat(1));
            h(4) = (t(1)+2*t0)*(t(1)^2+t(1)*t0);
            
            df(3,1) = 6*v0(1);
            dh(3,1) = t(1)^2+t(1)*t0+2*t(1)+t0;
            
            df(4,1) = 6*v0(2);
            dh(4,1) = t(1)^2+t(1)*t0+2*t(1)+t0;
        else
            f(3) = 6*(t(i-1)*t0*(t(i-1)+t0)*dp_long(i) - (t(i)+2*t(i-1)+t0)*t(i)*t0*dp_long(i-1) ...
                +t(i)*t(i-1)*(t(i)+t(i-1))*v0(1)*t0);
            h(3) = t(i)*t(i-1)*t0*(t(i)+t(i-1))*(t(i)+t(i-1)+t0);
            f(4) = 6*(t(i-1)*t0*(t(i-1)+t0)*dp_lat(i) - (t(i)+2*t(i-1)+t0)*t(i)*t0*dp_lat(i-1) ...
                +t(i)*t(i-1)*(t(i)+t(i-1))*v0(2)*t0);
            h(4) = t(i)*t(i-1)*t0*(t(i)+t(i-1))*(t(i)+t(i-1)+t0);
            
            df(3,i) = -6*(2*t(i)+2*t(i-1)+t0)*t0*dp_long(i-1) ...
                +6*(2*t(i)+t(i-1))*t(i-1)*v0(1)*t0;
            df(3,i-1) = 6*(2*t(i-1)+t0)*t0*dp_long(i)-6*2*t(i)*t0*dp_long(i-1)...
                +6*(t(i)+2*t(i-1))*t(i)*v0(1)*t0;
            dh(3,i) = t(i-1)*t0*(t(i)^2+t(i-1)*t(i)+(2*t(i)+t(i-1))*(t(i)+t(i-1)+t0));
            dh(3,i-1) = t(i)*t0*(t(i-1)^2+t(i-1)*t(i)+(2*t(i-1)+t(i))*(t(i)+t(i-1)+t0));
            
            df(4,i) = -6*(2*t(i)+2*t(i-1)+t0)*t0*dp_lat(i-1) ...
                +6*(2*t(i)+t(i-1))*t(i-1)*v0(2)*t0;
            df(4,i-1) = 6*(2*t(i-1)+t0)*t0*dp_lat(i)...
                -6*2*t(i)*t0*dp_lat(i-1)...
                +6*(t(i)+2*t(i-1))*t(i)*v0(2)*t0;
            dh(4,:) = dh(3,:);
        end
    end
    f(5) = vref*t(i)-dp_long(i);
    h(5) = t(i);
    
    df(5,i) = vref;
    dh(5,i) = 1;
    

    
    for j=1:5
        refy((i-1)*5+j) = -f(j)/h(j);
    end
    
    for j = 1:5
        nabla_t = df(j,:) - dh(j,:)*(f(j)/h(j));
        A((i-1)*5+j,1:nstep) = -nabla_t;
        A((i-1)*5+j,nstep+(i-1)*5+j) = -h(j);
        b((i-1)*5+j) = f(j)-dot(nabla_t,t);
        
        
%         nabla_t = df(j,:) - dh(j,:)*abs(f(j)/h(j));
%         A((i-1)*5+j+5*nstep,1:nstep) = nabla_t;
%         A((i-1)*5+j+5*nstep,nstep+(i-1)*5+j) = -h(j);
%         b((i-1)*5+j+5*nstep) = -f(j)+dot(nabla_t,t);
    end
    
    along(i) = - f(1)/h(1);
    alat(i) = - f(2)/h(2);
    jlong(i) = - f(3)/h(3);
    jlat(i) = - f(4)/h(4);
    vlong(i) = vref - f(5)/h(5) ;
end

varargout{1} = along;
varargout{2} = alat;
varargout{3} = jlong;
varargout{4} = jlat;
varargout{5} = vlong;
end