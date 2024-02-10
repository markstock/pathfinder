# pathfinder
C++ program to find optimal paths through heightfields

## Setup
Compile and run with:

    make
    ./pathfinder -h
    ./pathfinder -d SwitzerlandBerneseOberland.png -v 0.18
    ./pathfinder -d SpainOneArcsecond.png -v 0.1 -s 0 1500 -f 2999 1500
    ./pathfinder -d ScotlandSkye.png -v 0.018 -s 1875 2868 -f 558 1578 --hard ScotlandSkyeImpassable.png

Use `-d` to set the DEM (digital elevation model) and `-v` to set the slopes - the vertical distance between pure black and white divided by the horizontal (left-to-right) length of the represented area. For example, if your png were 1000 by 500 pixels and each pixel is 100 meters, then the width of your image is 100,000 meters. If black is -100m and white is 4900m, then the vertical range is 5000 meters. The coefficient for this case would be 0.05.

`-s` and `-f` set start and finish points, measured from top left (x then y). Finally, to include a map of impassable (or extra-difficult) terrain or water, use `-hard`.

## Goal
In order to create an organic civilization/city simulation engine which starts from the beginnings of habitation, we need a tool to find paths between two points on a scalar, raster field of passability. The first step being to simulate paths taken through a given area: from one boundary cell to another. When settlers arrive, paths between their camps and those boundary connections are then calculated. And when there are multiple settlers, paths between camps as well.

The goal of this code/library is not to determine where settlers encamp, but to accept a start and end pixel and a passability field and compute the path(s) between them.

Note that if uphill and downhill travel have equal costs for equal slope, the cost map is isotropic, and the direction of travel does not matter; but in the real world, the costs differ, so two separate paths might be found.

An additional tweak is that once a path is established (and presumed regularly used), it should affect the cost of traveling in that direction through that cell. Thus, subsequent pathfindings would preferentially use that existing "road."

Costs to traverse from a cell to one of its 8 neighbors would vary depending on the slope (high at both ends, low in the middle, lowest for a slight downward slope), the presence of water (rivers and lakes), and the presence of other, more time-varying surface features (woods, swamps, dangerous animal ranges). So, depending on the season (higher rivers, more undergrowth, active predators), the costs change and the routes might change. Regarding river crossings - with enough travel (and commensurate reduction in travel cost), there can be considered to be a bridge.

Thus, input could consist of multiple raster fields: one definitely with terrain elevation, but optional ones with rivers or vegetation or seasonal differences.

Output should be either to raster images to use as layers, or to a vectorized format (.seg).

Similar methods could be used to compute the range or effective property boundaries between a number of settlers on the same raster grid: each settler grows their land holdings at varying rates until running up against another. That may become another program.

## To Do
Implement command line arg to generate paths between N randomly-placed points, either pure random or metropolis random

Add command line args for the relative values of the cost functions: the minimum cost for terrain-only, the maximum cost addition for hard, the cost multiple for easy, etc.

Write out paths in .seg format for extra processing; make sure to include z component.

Allow method to optimally place a new point such that its cost to travel to a list of target points is minimized. Where roads exist, this might always choose a spot on a road or even an intersection of roads. That's how towns form, I suppose.

Consider the path-perpendicular slope in the cost. Path-tangential is slope, and we determine the cost of that properly; but path-perpendicular slope is also important. It's probably called "exposure" in climbing context. It's very hard to walk along steep slopes, even with no forward elevation gain/loss, unless there is a prepared trail carved into the slope.

This makes me think that there should be a parameter defining the width of a path compared to the size of a pixel. If a path is 0.01 to 0.1 of a pixel, then you can probably fit a switchback or two within a pixel; but not if closer to 1.0; and what would one even do if the parameter were > 2? That's probably when perpendicular slope makes a difference.

It would be cool to see a separate image or data plot for each path of elevation vs. distance. Maybe to `.path001.dat`?

Note that ASTER has a global water bodies database at https://search.earthdata.nasa.gov/search/granules?p=C1575734433-LPDAAC_ECS&pg[0][v]=f&tl=1686674313.39!3!!

## Credits
This code was written by Mark Stock <markjstock@gmail.com> in 2021.

## Citing Pathfinder

I don't get paid for writing or maintaining this, so if you find this tool useful or mention it in your writing, please please cite it by using the following BibTeX entry.

```
@Misc{Pathfinder2021,
  author =       {Mark J.~Stock},
  title =        {Pathfinder:  A C++ program to find optimal paths through heightfields},
  howpublished = {\url{https://github.com/markstock/pathfinder}},
  year =         {2021}
}
```
