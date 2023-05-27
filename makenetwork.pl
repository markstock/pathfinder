#!/usr/bin/perl
#
# manypaths.pl - find paths through terrain as it is being populated by random settlers
#

my $num = $ARGV[0];
my $dem = $ARGV[1];
my $opts = $ARGV[2];
# opts should be like "-v 0.0103 -c 0.001"

my $easyfile = "allpaths.png";
my $allpts = "finishpts";

# find the dimensions of the DEM
my $outline = `pngtopam ${dem} | pamfile -size`;
my @sizes = split(' ',$outline);
my $xsize = $sizes[0];
my $ysize = $sizes[1];
print "Input dem (${dem}) is ${xsize} x ${ysize}\n";

# generate a dummy easy file
$command = "convert -size ${xsize}x${ysize} canvas:black ${easyfile}";
print "${command}\n"; system $command;

my $basecommand = "./pathfinder -d ${dem} ${opts}";

# loop over all the settlers
for (my $i=0; $i<$num; $i=$i+1) {

  print "\nAdding point $i to network...\n\n";

  my $sx = int(rand($xsize));
  my $sy = int(rand($ysize));

  # do we have an existing "easy" file?

  # travel North and find optimal dest
  $command = $basecommand;
  if ($i > 0) { $command .= " -e ${easyfile}"; }
  if ($i > 0) { $command .= " --ff ${allpts}"; }
  $command .= " -s ${sx} ${sy} --fn --fs --fe --fw --op newpaths.png --om allmap.png";
  print "${command}\n"; system $command;

  # add current point to finish points file
  if ($i == 0) {
    open(FP,">${allpts}") or die "Can't open ${allpts}: $!";
  } else {
    # re-open the finish points file for appending
    open(FP,">>${allpts}") or die "Can't open ${allpts}: $!";
  }
  print FP "${sx} ${sy}\n";
  close(FP);

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
