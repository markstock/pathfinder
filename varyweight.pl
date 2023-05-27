#!/usr/bin/perl
#
# varyweight.pl - generate a series of images of paths with varying weights
#

my $minc = $ARGV[0];
my $maxc = $ARGV[1];
my $numfr = $ARGV[2];
my $cl = $ARGV[3];

# pathfinder -d UsaAzGCwide_v0p0104.png -v 0.0104 -c 0.001 -s 3095 1571 --fs --fn --fe --fw --om UsaAzGCwide_c0p001.png
my $basecommand = "./pathfinder ${cl} --fs --fn --fe --fw";

# find log ranges
my $minlog = log($minc);
my $maxlog = log($maxc);
my $dxlog = ($maxlog - $minlog) / ($numfr - 1.0);

# loop over all the settlers
for (my $i=0; $i<$numfr; $i=$i+1) {
  my $logc = $minlog + $i * $dxlog;
  my $cval = exp($logc);

  my $outfn = sprintf("out_%04d.png",$i);

  # travel North and find optimal dest
  my $command = $basecommand." -c ${cval} --om ${outfn}";
  print "${command}\n"; system $command;
}
