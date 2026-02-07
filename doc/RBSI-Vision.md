# Az-RBSI Vision Integration

This page includes detailed steps for integrating robot vision for your
2026 REBUILT robot.

--------

### PhotonVision

The preferred method for adding vision to your robot is with [PhotonVision](
https://photonvision.org/).  This community-developed open-source package
combines coprocessor-based camera control and analysis with a Java library
for consuming the processed targeting information in the robot code.

#### Recommended Setup with Az-RBSI

We recommend using Arducam [OV9281](https://www.amazon.com/dp/B096M5DKY6)
(black & white) and/or [OV9782](https://www.amazon.com/dp/B0CLXZ29F9) (color)
cameras for robot vision due to their Global Shutter, Low Distortion, and USB
connection.  In addition to the lens delivered with the camera, supplementary
lenses may be purchased to vary the FOV available to the detector for various
robot applications, such as [Low-Distortion](
https://www.amazon.com/dp/B07NW8VR71) or [General Purpose](
https://www.amazon.com/dp/B096V2NP2T).

For the coprocessor that controls the cameras and analyzes the images for
AprilTag and gamepiece detection, we recommend using one or two Orange Pi 5
single-board computers -- although PhotonVision does support a number of
[different coprocessor options](
https://docs.photonvision.org/en/latest/docs/quick-start/quick-install.html).
As decribed in the [Getting Started Guide](RBSI-GSG.md), we include a 3D print
for a case that can hold one or two of these computers.

#### Setting up PhotonVision on the Coprocessor

Download the appropriate [disk image](
https://github.com/PhotonVision/photonvision/releases/tag/v2026.2.1) for your
coprocessor and burn it to an SD card using the [Raspberry Pi Imager](
https://www.raspberrypi.com/software).  Connect the powered-on coprocessor
to the Vivivid radio (port 2 or 3) via ethernet, or connect to a network switch connected to the radio via ethernet, and connect to the PhotonVision software
at the address ``http://photonvision.local:5800``.

Before you connect the coprocessor to your robot, be sure to set your team
number, set the IP address to "Static" and give it the number ``10.TE.AM.11``,
where "TE.AM" is the approprate parsing of your team number into IP address,
as used by your robot radio and RoboRIO.  If desired, you can also give your
coprocessor a hostname.

![PhotonVision Network Settings](PV_Network.png)

We suggest you give your first coprocessor the static IP address
``10.TE.AM.11``, and your second coprocessor (if desired) ``10.TE.AM.12``.
The static address allows for more stable operation, and the these particular
addresses do not conflict with other devices on your robot network.

Plug in cameras (two or three per coprocessor) and navigate to the Camera
Configs page (see below).  Activate the cameras.

![PhotonVision Camera Configs](PV_Cameras.png)

#### Configuring and Calibrating your Cameras

This is the most important part, and the description of which is left as an
exercise for the user.

Good luck!

#### Using PhotonVision for vision simulation

This is an advanced topic, and is therefore in the Restricted Section.

![Restricted Section](restricted_section.jpg)
