#!/usr/bin/perl
#
# crosspaths.pl - find a set of optimal roads crossing a DEM N-S and E-W
#

my $dem = $ARGV[0];
my $vscale = $ARGV[1];

# find the dimensions of the DEM
my $outline = `pngtopam ${dem} | pamfile -size`;
my @sizes = split(' ',$outline);
my $xsize = $sizes[0];
my $ysize = $sizes[1];
print "Input dem (${dem}) is ${xsize} x ${ysize}\n";

my $basecommand = "./pathfinder -d ${dem} -v ${vscale}";

# do N-S road first
my $sx = int($xsize/2);
my $sy = 0;
while (1) {
  my $lastsx = $sx;

  # travel North and find optimal dest
  $sy = $ysize-1;
  my $command = $basecommand." -s ${sx} ${sy} --fn --op ns.png";
  print "running ($command)\n";
  my $results = `${command} | grep \"finish point\"`;
  # (finish point is at 1296 0)
  my @tokens = split(' ',$results);

  # travel South and find optimal dest
  $sx = $tokens[4];
  $sy = 0;
  $command = $basecommand." -s ${sx} ${sy} --fs --op ns.png";
  print "running ($command)\n";
  $results = `${command} | grep \"finish point\"`;
  @tokens = split(' ',$results);

  $sx = $tokens[4];
  last if ($sx == $lastsx);
}

# now optimize E-W road
$sx = 0;
$sy = int($ysize/2);
while (1) {
  my $lastsy = $sy;

  # travel West and find optimal dest
  $sx = $xsize-1;
  my $command = $basecommand." -s ${sx} ${sy} --easy ns.png --fw --op nsew.png";
  print "running ($command)\n";
  my $results = `${command} | grep \"finish point\"`;
  # (finish point is at 1296 0)
  my @tokens = split(' ',$results);

  # travel East and find optimal dest
  $sx = 0;
  $sy = $tokens[5];
  $command = $basecommand." -s ${sx} ${sy} --easy ns.png --fe --op nsew.png";
  print "running ($command)\n";
  $results = `${command} | grep \"finish point\"`;
  @tokens = split(' ',$results);

  $sy = $tokens[5];
  last if ($sy == $lastsy);
}

