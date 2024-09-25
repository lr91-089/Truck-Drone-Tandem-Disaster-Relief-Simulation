#!/bin/bash

file_name="drone_truck_disaster_linux.out"


cd /home/rocha01/source/drone_truck_disaster_linux

for csvfile in /home/rocha01/source/drone_truck_disaster_linux/paper_sichtfeld_experiments_bash/*.csv;do
   (
   exec < $csvfile

   while read line
   do
      echo "Record is : $line"
      ./$file_name $line
   done
   )&
done

wait

echo "All CSV files have been processed."
