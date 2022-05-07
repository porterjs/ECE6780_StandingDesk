# HMI Design

## Quick Tip

The key to HMI design is to make it a "dumb" operator.  All the "smarts" should be programmed into the MCU.  For instance, timers and counters may be initiated by an HMI, but the MCU should perform/monitor the calculations.  

## Software Requirements

The HMI requires the Nextion Editor.

## Navigation

The HMI can navigate between different "pages" or "screens" which can be used to organize the contents of the user interface.  

| Page | Content           |
| ---- | ----------------- |
| 0    | Main Page         |
| 1    | Operator Controls |
| 2    | Monitor / Status  |
| 3    | Configuration     |
| 4    | Scheduler         |

### Main Page

- [x] Fun team logo
- [x] Main controls (up/down)
- [x] Main status (desk moving up/down, fault)

### Operator Controls

- [x] Manual Control (Up/Down Buttons)
- [x] Home/recalibrate
- [x] Goto/specify height
- [ ] Manual Speed

### Monitor / Status

- [x] MCU heartbeat
- [ ] Current height of each desk leg
- [x] Tilt angle of desk surface
- [ ] Motor current

### Configuration

- [ ] State Space gains
- [ ] Max Speed 1 (Rabbit)
- [ ] Max Speed 2 (Turtle)
- [ ] Homing Speed
- [ ] Standing Height SP
- [ ] Sitting Height SP

### Advanced Configuration

- [ ] State Space gains
- [ ] State Space matrices
- [ ] Move one leg at a time manually

### Scheduler

- [ ] Enable Scheduler
- [ ] Set intervals (e.g. 1 hour) 
- [ ] NOTE: Buzzer sounds twice 5 seconds before transition

Buzzer sounds once for min/max limit





