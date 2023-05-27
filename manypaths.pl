#!/usr/bin/perl
#
# manypaths.pl - find paths through terrain as it is being populated by random settlers
#

my $dem = $ARGV[0];
my $vscale = $ARGV[1];
my $num = $ARGV[2];
#my $easyfile = $ARGV[3];
my $easyfile = "allpaths.png";

# find the dimensions of the DEM
my $outline = `pngtopam ${dem} | pamfile -size`;
my @sizes = split(' ',$outline);
my $xsize = $sizes[0];
my $ysize = $sizes[1];
print "Input dem (${dem}) is ${xsize} x ${ysize}\n";

# generate an easy file?
$command = "convert -size ${xsize}x${ysize} canvas:black ${easyfile}";
print "${command}\n"; system $command;

my $basecommand = "./pathfinder -d ${dem} -v ${vscale} -c 0.001";

# loop over all the settlers
for (my $i=0; $i<$num; $i=$i+1) {
  my $sx = int(rand($xsize));
  my $sy = int(rand($ysize));

  # do we have an existing "easy" file?

  # travel North and find optimal dest
  $command = $basecommand;
  if ($i > 0) { $command .= " -e ${easyfile}"; }
  $command .= " -s ${sx} ${sy} --fn --fs --fe --fw --op newpaths.png --om allmap.png";
  print "${command}\n"; system $command;

  # finally merge the easy file and the new one
  my $command = "pngtopam ${easyfile} > .tempold.pam";
  print "${command}\n"; system $command;
  $command = "pngtopam newpaths.png > .tempnew.pam";
  print "${command}\n"; system $command;
  $command = "pamarith -maximum .tempold.pam .tempnew.pam | pamtopng > allpaths.png";
  print "${command}\n"; system $command;
  $easyfile = "allpaths.png";
}
unlink ".tempold.pam", ".tempnew.pam", "newpaths.png";
