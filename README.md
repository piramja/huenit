# huenit
custom scripts for controlling the Huenit robot arm (https://huenit.imweb.me/). The port on which the robot is connected should be discovered automatically by the scripts. They should run both on linux and windows.


## Installation

install dependencies (pyserial):

```
pip install pyserial
```

## huenit_jog_control.py

simple keyboard jog control. Just start the script and start jogging. Theres some parameters you can play with in the script.

## huenit_teach_replay.py

fast way of teaching robot movements by physically moving the robot joints, recording each position by pressing 'c' on the keyboard. You can then replay all recorded movements by pressing 'enter'. Records can be saved to a file and loaded from there.
