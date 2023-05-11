#!/bin/bash

# Cut each line by the ":" delimiter

echo "Before: "
cat ./denavit-generated.txt

temp_file=$(mktemp)
temp_file2=$(mktemp)

# Separates the results in table with d, theta, r (a) and alpha cols
cut -d ":" -f 2 ./denavit-generated.txt | column --table --separator , > $temp_file

# With awk, print all the cols, but remove the "variable=" before and the "[x]"  after each col
awk 'BEGIN {print "d\ttheta\ta\talpha"} {print $1,$3,$5,$7}' $temp_file | sed 's/\w*=//g' | column --table -o , > $temp_file2

# Remove all spaces
sed 's/ //g' $temp_file2 > "denavit-generated.csv"

echo "After: "
cat denavit-generated.csv

# Clean up
rm $temp_file
rm $temp_file2

