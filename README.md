# Sensor_D6100
This is a MySensor program to control, interface with an APEX Destiny 6100(AN) Alarm panel.  This is an older alarm panel, but its capability exceed what can be found on current Honeywell-Ademco panels.

Apex was bought by Ademco who was bought by Honeywell. Its is NOT compatible with current Honeywell-Ademco panels, but has the framework to be adapted.

My system has 63 active zones, most are using wireless Ademco-5800 series sensors

It read and parses serial data from the alarm and format them into MySensor messages.

   This is running on a MoteinoMEGA, and connect to the D6100 via a TTL serial interface.
   
   Radio is 915MHz RFM69HW.
   
   Tested with MySensor 2.0.0 beta.
   
   Tested with D6100 firmware 8.07.
