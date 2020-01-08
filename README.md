# Dog Door / Pet Door Project

This project is a work in progress, aiming to provided an automated door for our dog and cats.

With the Particle Photons built in WiFi, I plan to integrate in the monitoring of the door with existing Home Automation.

*TODO - Add Frizting Diagram*

## Connectivity

The status of the various switches, sensors and other internal variables are exposed via Particle Cloud. This also provides some basic over-ride capabilities, including a remote reset.

MQTT support has also been added to report state back to an MQTT Broker. The device also accepts some basic commands via MQTT, for example to open the door. This has been tested with HomeAssistant.io.

## Potential TODO List

* Experiment with ISO11784 134.2K RFID Module to read RFID tags in our pets. As an alternative to using the infrared sensor outside.
* Investigate locking mechanisms, solenoid?

## Operation

The manual toggle switch (on-off-on) is used to provide 2 fixed states, keep open and keep closed. In the centre off position the door can be opened by pressing the manual push button, or when either an external or internal infrared sensor detects a presence closer than a defined threshold.

If the desired state of the door is Open, then the motors will run until the top limit switch is reached, lifting the Perspex door up. Once reached the door will stay open for 10 seconds, or longer if any of the sensors are triggered, resetting the timer each time. The door then returns to the default closed states and disables the stepper motors (saving power). Until needed again.

## Hardware

* Structure:
  * 2040 and 2020 Aluminium extrusion attached to plywood backing board
  * Various Aluminium pieces and fixings. 
* Door:
  * A2 (594 x 420mm) 5mm thick clear Perspex sheet, using slot in Aluminium as a guide.
  * A2 (594 x 420mm) 5mm PVC Foamex board, trimmed slightly narrower, sitting in front on the clear Perspex for a little added insulation.
* Mechanical:
  * 2 x NEMA 17 motor mount
  * 2 x Idler pully mounting plate
  * 2 x Delrin smooth idler pulley
  * 2 x Limit switch mounting brackets
  * 2 x 6mm GT2 timing belts
  * 2 x GT2 20T drive gear
  * 2 x MGN12 Linear Slide Rails 600mm and MGN12H blocks
  * 2 x 3D Printed (Thanks John Crawford) parts to attach belt to sliding rail blocks
* Electrical:
  * Particle.io Photon microcontroller
  * 2 x Longrunner 1.7A NEMA 17 stepper motors
  * Longrunner TB6600 stepper motor driver
  * 12V 5A DC Power Adapter
  * LM2596 3A adjustable step down DC-DC converter power supply (for 5V supply)
  * 2 x Sharp GP2Y0A21 10-80cm Infrared proximity/distance sensors
  * 2 x Microswitch V4 type with roller or R-shape lever
  * Toggle switch (on-off-on)
  * Push button
  * Various wires and connectors
