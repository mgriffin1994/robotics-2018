#!/usr/bin/env bash

DATA="beacon_data.txt"
OBS="obs_data.txt"
FORMATTED="beacon_data_formatted.txt"
SAVED="beacon_data.npz"

# Copy data over
scp nao@10.202.16.61:"/home/nao/beacon_data.txt" "beacon_new_data.txt"
scp nao@10.202.16.61:"/home/nao/obs_data.txt" "obs_new_data.txt"

# Combine
cat beacon_new_data.txt >> "$DATA"
cat obs_new_data.txt >> "$OBS"

# Format data in csv
awk 'NR%15 {printf("%s, ", $0); next} {print $0}' $DATA > $FORMATTED

# Save to npz
python -c "import numpy as np; df = np.loadtxt(\"$FORMATTED\", delimiter=\",\"); np.savez(\"$SAVED\", actions=df[:,:3], observations=df[:,3:])"
