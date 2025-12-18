

**THE ORIGINAL GOAL**
**Make a 16mm film projector portable and playable like an instrument. Retrofit with a DC motor and LED lamp, controlled by ESP32 micro-controller.**
This project is part of [SPECTRAL Wandering Sounds and Images Seminar](https://www.mire-exp.org/evenement/spectral-wandering-sound-and-images/) in Nantes FR in 2023/2024, enabling film projection performances that explore itinerancy, nomadism, and adaptation to multiple environments.

----

Contents
======

---- 

 - [Bill of Materials (BOM)](https://www.filmlabs.org/wiki/en/meetings_projects/spectral/mire-wandering/wandering-16mmprojection/budget) (filmlabs.org) or [Google Sheets alternate](https://docs.google.com/spreadsheets/d/1z_asHddtIuv7a7RkZ9WCBqF81oMiJ9spwqh7lCQfW5E)
//self reminder to update w/ off the shelf parts and harvestable components
	
 - [Step-by-step Construction Guide](https://www.filmlabs.org/wiki/en/meetings_projects/spectral/mire-wandering/wandering-16mmprojection/construction_guide/start) for building and using the projector (filmlabs.org)
//break out into bulleted list with additional source for checking + installing LTC3780 and fan
//for non internal battery installations describe fitting of 20amp power supply
//describe optical printer dock and pros + cons
//describe stiffer motor mount construction from thick acrylic + pros and cons to using printed materials for this section

[openscad](https://github.com/z-l-p/film-projector-retrofit/tree/main/openscad)
----

 - Editable design files for 3D-printed parts (in OpenSCAD format)

[projector\_code](https://github.com/bpatern/EIKII_multifunctional_artsprojector/tree/main/projector_code)
----

 - Code for the ESP32 micro-controller, raspberry Pi Pico 2W instruction parsing controller, and optical printer code that controls the projector (Arduino IDE)
 - optical printer functionality uses [Matt McWilliams' mCopy](https://github.com/sixteenmillimeter/mcopy) for sequencing. led support is enabled in sort of a hackneyed way, that currently only really works with my setup. id like to be able to have mCopy recognize the LED and its brightness control but I can't get mCopy and the led control scheme used in this code to shake hands. 

[stl](https://github.com/z-l-p/film-projector-retrofit/tree/main/stl)
----

 - Printable files for 3D-printed parts (rendered from openscad)



