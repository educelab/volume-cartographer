#!/usr/bin/perl
use warnings;

#used to run multiple landmark processes at once 

#die "Usage: \$ perl $0 program batchfile\n" if scalar @ARGV != 2;

my $programme = $ARGV[0];
my $arguments = $ARGV[1];

die "Batchfile must be csv\n" if $arguments !~ /\.csv$/;
die "$arguments does not exist\n" if not defined -e $arguments;

open(my $file, "<", $arguments) or die "couldn't open $arguments";

my $command;
for (<$file>) {
  chomp;
  s/\s*,\s*/ /g;
  $command .= " '$programme $_'"
}

system("parallel -j+0 :::$command")
