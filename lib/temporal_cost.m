function fun = temporal_cost(t,path,theta,vref,Q,v0,t0)
refy = t2y_new(t,path,theta,vref,v0,t0);
fun = [t;refy]'*Q*[t;refy];
end