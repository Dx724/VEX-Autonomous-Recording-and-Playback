# VEX Autonomous Recording and Playback
 Extracted from VEX Turning Point Code of Team 9932E
 
 *Using PROS v3.1.5, a microSD card is required*
 
 With the release of the new generation of VEX sensors, I thought it was finally time to publish this code from 2019. Included in it is the often-sought-after ability to record autonomous routines on the fly. Simply enter driver control, use a controller to perform the desired actions, and then press the center button on the V5 screen in order to save the recorded routine to the SD card. The saved autonomous path is present in a file on the SD card and can be backed up for future use or renamed to allow multiple different autonomous routine choices which are selected during the pre-autonomous period. The working principle behind this code is that controller inputs are logged and then saved in a format which can be loaded later and replayed. Thus, the majority of robot control code should be placed within dataHandler.cpp, so that updates to functionality will propogate to both driver and autonomous routines.

Notable features:
- Gyroscope-based autonomous platform climbing
- Software-based flywheel double shot (hitting high and low flags in quick succession by taking advantage of the first shot's effect on flywheel speed)
- Vision sensor alignment task
- Motor braking (to make the robot harder to push)
- SD-card-based parameter control (so that code does not have to be reuploaded to modify common constants such as team color or flywheel speed)
- Autonomous selection interface

Interfaces to generate autonomous paths graphically and edit saved autonomous routines were also created, so send me an email if those would be of assistance to your team.

Hopefully the release of this code will inspire further innovations while also leveling the playing field for teams which cannot afford to purchase the new hardware.

- David, Team 9932E
