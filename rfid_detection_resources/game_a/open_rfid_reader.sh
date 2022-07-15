#!/bin/bash

echo 0 > control.txt
rm results_001625143965.txt
touch results_001625143965.txt

java -jar 001625143965.jar &
