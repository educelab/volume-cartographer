#!/usr/bin/perl
use warnings;
use strict;
use File::Which;

die "Usage: \$ perl $0 program batchfile\n" if scalar @ARGV != 2;
my $programme = $ARGV[0];
my $arguments = $ARGV[1];
die "Batchfile must be csv\n" if $arguments !~ /\.csv$/;
die "$arguments does not exist\n" if not defined -e $arguments;

my @programme = which($programme);
if (scalar @programme == 0) {
  if (not -x $programme) {
    die "$programme is not executable\n"
  } elsif (not defined -e $programme) {
    die "$programme does not exist and is not in search path\n"
  }
}

open(my $file, "<", $arguments) or die "couldn't open $arguments";

for (<$file>) {
  chomp;
  s/\s*,\s*/ /g;
  system("$programme $_");
}
