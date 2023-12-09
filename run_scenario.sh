#!/bin/bash

source ~/.bashrc
cd /home/erdos/workspace/scenario_runner
python3 scenario_runner.py  --scenario $1 --reloadWorld --timeout 100000 --output
