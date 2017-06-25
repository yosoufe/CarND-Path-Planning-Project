reset
set size 1,1

set multiplot

#first
set size 1,0.5
set origin 0,0.5
plot 'NIS_lidar' with lines

#second
set origin 0,0
plot 'NIS_radar' with lines

unset multiplot
