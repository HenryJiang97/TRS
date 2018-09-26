function traj=traj_planning(object_x,object_y,start_x,start_y)
%for example traj_planning(-2.5,2,-3,-5)
global cellsize;
cellsize = .25;

mapFig = figure(1);
axis equal;
axis([-7.8 7.8 -7.8 7.8]);
drawnow;

% House map. Has 0 for free space and 1 for obstacles.
load('map');
% House map, with all obstacles dilated by one extra cell.
load('fmap');

goal = [ ij(object_x), ij(object_y) ];
dx = DXform(double(fmap), 'private');
dx.plan([goal(1); goal(2)]);
figure(2);
dx.plot();
 figure(1);
traj = dx.path([ij(start_x) ; ij(start_y)])';

% Convert trajectory to world coordinates:
traj = [ xy(traj(1,:)); xy(traj(2,:))];
[X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
plot(X(map==1), Y(map==1), '*r', 7.5, 0, 'or', 0, 7.5, 'og',traj(1,:), traj(2,:), 'b');

end