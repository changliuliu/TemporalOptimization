function handle = show_vehicle(p,theta,varargin)
if ~isempty(varargin)
    color = varargin{1};
else
    color = 'b';
end
width = 1.8;
length = 4.5;
corners = zeros(2,5);
vlong = [cos(theta);sin(theta)];
vlat = [sin(theta);-cos(theta)];
corners(:,1) = p + vlong*length/2 + vlat*width/2;
corners(:,2) = p + vlong*length/2 - vlat*width/2;
corners(:,3) = p - vlong*length/2 - vlat*width/2;
corners(:,4) = p - vlong*length/2 + vlat*width/2;
corners(:,5) = corners(:,1);
handle.edge = plot(corners(1,:),corners(2,:),'k');
handle.fill = fill(corners(1,:),corners(2,:),color);
end