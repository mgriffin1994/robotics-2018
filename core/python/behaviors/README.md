NAOSAMI
=================

All behavior and math files are in ```core/python/behaviors/```.

1. In the tool, run the ```naosami``` behavior to collect real world data.
2. Run the ```./format_sh``` script in the ```~/nao/trunk/beacon_data_sim/``` folder to convert to a compressed 
```numpy``` format.
3. Run the ```do_maths.py``` Python script to run the EM algorithm on the compressed files.
4. For the action interpolation, run the ```interpolate_actions.py``` Python script on the saved outputs of 
the ```do_maths.py``` script.

For collecting real data, change the ```do_maths.py``` and ```./format.sh``` to point to 
```~/nao/trunk/beacon_data/``` folder and the files in that folder.
