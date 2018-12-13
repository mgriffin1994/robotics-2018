NAOSAMI
=================

All behavior and math files are in ```core/python/behaviors/```. The data collection files are in the top-level directories `beacon_data/` and `beacon_data_sim/`.

**Notable files**:
 - core/python/behaviors/do_maths.py
   - runs the EM algorithm on the collected training data
 - core/python/behaviors/naosami.py
   - collects the training data by randomly choosing actions and observing beacon heights
 - core/python/behaviors/interpolate_actions.py
   - interpolates the action table using K-nearest neighbors and linear interpolation
 - beacon_data/obs_data.txt
   - holds statistics on the collected training data such as the number of total actions executed
   - the number of unique actions (out of 40) executed
   - the number of times each beacon has been seen
   - the percentage of frames where the Nao saw all NaNs (no beacons)

**Instructions on how to run our code**:
1. In the tool, run the ```naosami``` behavior to collect real world data.
2. Run the ```format_data.sh``` script in the ```beacon_data_sim/``` top-level folder to convert to a compressed
```numpy``` format.
3. Run the ```do_maths.py``` Python script to run the EM algorithm on the compressed files.
4. For the action interpolation, run the ```interpolate_actions.py``` Python script on the saved outputs of
the ```do_maths.py``` script.

**Miscellaneous Notes**:
 - For collecting real data, change the ```do_maths.py``` and ```format_data.sh``` to point to ```beacon_data/``` the top-level folder and the files in that folder.

