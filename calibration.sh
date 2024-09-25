#!/usr/bin/env bash
cd "$(dirname "$0")"


file_name="drone_truck_disaster_linux.out"


for csvfile in $(dirname "$0")/paper_calibration_bash_files/*.csv;do
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
