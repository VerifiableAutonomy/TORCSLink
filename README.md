# Simulink Robot for The Open Source Race Car Simulator (TORCS)
This robot plugin for [TORCS](http://torcs.sourceforge.net/) allows you to develop a vehicle control system in MATLAB and Simulink. An example Simulink model
is provided which demonstrates an Automatic Cruise Control system. 

Currently, Windows is the only supported platform, but a Linux version is under development...

## Set up
1. Download the TORCS source files and compile them as detailed
[here](http://torcs.sourceforge.net/index.php?artid=3&name=Sections&op=viewarticle#linux-src-all-win) 
(Note, there may be some minor bug fixes needed depending on your version of Visual Studio)

2. Checkout this repository in the `src/drivers` folder

    ```
    cd path/to/torcs/src/drivers
    git clone https://github.com/VerifiableAutonomy/TORCSLink.git
    ```
    
3. Add the `matlab.vcxproj` project to your TORCS solution
4. For the initial build you need to disable the `TL_USE_DOUBLE_POSITION` and `TL_ENABLE_RESTARTS`
features by commenting out the `#define` lines in `TORCSLink.h`, these can be enabled later with reference to the sections
below

5. Build the solution. TORCSLink is now ready to go...

## Usage
This section tells you how to get up and running with the Automatic Cruise Control example provided. This has been tested
with MATLAB R2014b (64-bit version).

1. To use the provided Automatic Cruise Control example it is recommended to install the *Motorway* track
    (a large oval with long straights and shallow corners). To do so, simply copy the `Motorway` folder
    in to the `runtime/tracks/road` directory in your TORCS directory.

2. Start TORCS and select `Race` > `Quick Race` > `Configure Race`, choose the `Motorway` track. Deselect any drivers 
    which may be present in the race, then add the drivers `matlab 0` and `matlab 1`.

3. Select `New Race`. In the command line output you should see 

    ```
    matlab 0: Online
    matlab 1: Online
    ```
This indicates that TORCSLink is ready for Simulink to connect.

4. Open the `ACC_Example.slx` model in Simulink and run it. In TORCS, you should see the vehicles set off and drive the profile 
    contained in the model. After the model is finished (90s), the cars will retain their previous control values and crash in
    to something! You should restart the race before executing the model again (or configure this to happen automatically, see below).

## Configure additional features
### Automatic restarts
The initialisation function called by the Simulink model attempts to restart the race automatically, such that all executions of the
model start from a consistent state. To enable TORCS to service this request a modification must be made to the TORCS source code.
You can find the details of this modification [here](http://torcs.sourceforge.net/index.php?name=Sections&op=viewarticle&artid=30#c6_8).
Once you have made this change you should uncomment the `#define TL_ENABLE_RESTARTS` line in `TORCSLink.h` and rebuild the solution.

### Double precision position data
By default, TORCS stores the entire vehicle state as a single precision `float` which is sufficient for the vast majority of
purposes. When designing multi-vehicle control techniques, however, a common condition encountered is that where two vehicles
experience near identical velocities. If we operate these vehicles on fairly large tracks, where the position values can reach
in to the 1000's, we only get millimeter accuracy from single precision calculation. TORCS integrates velocity to position at 500Hz
so when the velocities of two vehicles are similar we can get to the situation where the distance between two vehicles will 
be calculated as constant due to the limited resolution of the `float`.

To illustrate this by way of an example, assume we have two vehicles both moving parallel to the x-axis, one with an initial 
position of 1500m and the other 1490m (a distance of 10m apart). If the lead vehicle is travelling at 14m/s and the follower
at 14.01m/s. Intuitively, we can see that after 1s we would expect the leader to be at 1514m and the follower to be at 1504.01m
(a distance of 9.99m apart). But lets perform this calcultion the way TORCS does it, by integrating the velocities at 500Hz
(i.e. over a period of 0.002s). At the first time step...

Vehicle 1: 1500 + 14x0.002 = 1500.028

Vehicle 2: 1490 + 14.01x0.002 = 1490.028*02*

The final two digits of Vehicle 2's position will be lost due to the precision of a `float`, so if we continue this integration
for an entire second we would get have vehicle 1 in its correct location of 1514m but vehicle 2 would be at 1504m (still at 10m
spacing). This doesn't happen in practice because the velocities are rarely held constant for long periods, however the limited
precision does cause quantisation to occur when calculating the distance between two vehicles with similar velocity. This can
be seen in the cruise control example provided by zooming in on the Spacing scope signal towards the end of the simulation (~70-90s)
and observing the noise in the signal. When more vehicles are introduced, the effect can get more pronounced, so it should be
mitigated.

To modify the TORCS source code to fully represent position as a double precision value requires a significant number of changes.
Therefore, I propose a small hack to simply calculate the position as a double precision value in addition to the single precision
value. To do this we need to modify two files in the TORCS source code, patch files to do this are provided (`doublePosition_car.patch`
and `doublePosition_tgf.patch`), these must first be placed in the TORCS root directory, then applied with

    patch src/modules/simu/simuv2/car.cpp < doublePosition_car.patch
    patch export/include/tgf.h < doublePosition_tgf.patch

Finally, enable the use of double precision position values in TORCSLink by uncommenting `#define TL_USE_DOUBLE_POSITION` in
`TORCSLink.h`, then rebuild the solution. 

Re-run the example and observe the noise in the spacing signal is no longer present.
