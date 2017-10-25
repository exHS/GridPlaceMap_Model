# GridPlaceMap_Model
This is a simple simulation of the GridPlaceMap Model described in the paper of _Pilly et. al 2012_.

To execute the simulation just run the main.m file in matlab.

The agent is located in a circular environment in which it has to navigate towards an activated cue. Once it reaches that cue the episode is over and another cue is choosen from a list. 

For now only _Stripe Cells_ are implemented. The tuning parameters for the ring attractors and therefore for the different stripe cells can be set in the __HPC__ class. The default values are taken from the paper  of _Pilly et. al 2012_.

### Displaying 

One can choose between two different displaying modes in the __HPC__ class:
 * DISPLAY_MODE = 1 : the environment together with the rate map of a defined stripe cell are displayed. One can choose which stripe cell should be recorded with the variable _STRIPE_CELL_ = {direction,scale,phase}.
 * DISPLAY_MODE = 2 : the environment together with the rate map and all ring attractors are displayed. 	__IMPORTANT__: If displaying all ring attractors one should increase the number of phases to display e.g. _PHASES_ =  [0:1/100:1]

For faster generation of the rate map it is recommended to set the _FAST_PLOTTING_ variable in the __main.m__ file to 1. By that the figures are updated only once an episode has finished i.e. a goal is reached. 