# Maps

Saved occupancy-grid maps live here. CP-C1 (mapping) writes them:

```bash
ros2 run nav2_map_server map_saver_cli -f <this dir>/workshop_map
```

That produces `workshop_map.yaml` + `workshop_map.pgm`. `navigation.launch.py`
defaults to `map:=workshop_map.yaml` in this directory, so saving it here means
CP-C2/C3 work with no extra arguments.
