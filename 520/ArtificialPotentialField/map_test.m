
map = loadmap('map_3.txt');
obstalce = map.obstacles;
bound = map.boundary;

lynxStart();
plotmap(map, 3)
hold on

q = [-1 0.8 -0.2 -0.3 0 0];

lynxServo(q);
