# pathfinder
C++ program to find optimal paths through heightfields

## Setup
Compile and run with:

    make
    ./pathfinder -h
    ./pathfinder -d SwitzerlandBerneseOberland.png -v 0.18
    ./pathfinder -d SpainOneArcsecond.png -v 0.1 -s 0 1500 -f 2999 1500

The first argument is the DEM (digital elevation model) and the second sets the slopes - it is the vertical distance between pure black and white divided by the horizontal (left-to-right) length of the represented area. For example, if your png were 1000 by 500 pixels and each pixel is 100 meters, then the width of your image is 100,000 meters. If black is -100m and white is 4900m, then the vertical range is 5000 meters. The coefficient for this case would be 0.05.

## Goal
In order to create an organic civilization/city simulation engine which starts from the beginnings of habitation, we need a tool to find paths between two points on a scalar, raster field of passability. The first step being to simulate paths taken through a given area: from one boundary cell to another. When settlers arrive, paths between their camps and those boundary connections are then calculated. And when there are multiple settlers, paths between camps as well.

The goal of this code/library is not to determine where settlers encamp, but to accept a start and end pixel and a passability field and compute the path(s) between them.

Note that if uphill and downhill travel have equal costs for equal slope, the cost map is isotropic, and the direction of travel does not matter; but in the real world, the costs differ, so two separate paths might be found.

An additional tweak is that once a path is established (and presumed regularly used), it should affect the cost of traveling in that direction through that cell. Thus, subsequent pathfindings would preferentially use that existing "road."

Costs to traverse from a cell to one of its 8 neighbors would vary depending on the slope (high at both ends, low in the middle, lowest for a slight downward slope), the presence of water (rivers and lakes), and the presence of other, more time-varying surface features (woods, swamps, dangerous animal ranges). So, depending on the season (higher rivers, more undergrowth, active predators), the costs change and the routes might change. Regarding river crossings - with enough travel (and commensurate reduction in travel cost), there can be considered to be a bridge.

Thus, input could consist of multiple raster fields: one definitely with terrain elevation, but optional ones with rivers or vegetation or seasonal differences.

Output should be either to raster images to use as layers, or to a vectorized format (.seg).

Similar methods could be used to compute the range or effective property boundaries between a number of settlers on the same raster grid: each settler grows their land holdings at varying rates until running up against another. That may become another program.

## Credits
This code was written by Mark Stock <markjstock@gmail.com> in 2021.
