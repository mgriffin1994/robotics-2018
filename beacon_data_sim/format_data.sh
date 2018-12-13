#!/usr/bin/env bash

DATA="beacon_data_sim.txt"
OBS="obs_data_sim.txt"
FORMATTED="beacon_data_formatted.txt"
SAVED="beacon_data.npz"

# Format data in csv
awk 'NR%15 {printf("%s, ", $0); next} {print $0}' $DATA > $FORMATTED

# Save to npz
python -c "import numpy as np; df = np.loadtxt(\"$FORMATTED\", delimiter=\",\"); np.savez(\"$SAVED\", actions=df[:,:3], observations=df[:,3:])"