#!/bin/bash

source ~/.bashrc
cd /home/erdos/workspace/scenario_runner
PYTHONPATH=/home/erdos/workspace:$PYTHONPATH python3 scenario_runner.py --scenario $1 --reloadWorld --timeout 10000 --output
