^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teraranger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2019-02-11)
------------------
* Prevent unintended modification on temp_array when clipping values
* Contributors: Marcin Pilch, Pierre-Louis Kabaradjian

2.0.0 (2018-12-06)
------------------
* Update roswiki and Readme file for Evo Thermal
* Fix error in Evo64 px parsing
* Add frame_id in rgb image message
* Change topic naming
* Fix rosparam default value type
* Prevent header desynchronization in Thermal driver
* Add CRC check to ACK reading
* Prevent node from sending VCP commands to UART backboard
  Based on baudrate speed
* Refactor Evo 64px driver
* Remove fps estimation in Evo 64px driver
* Rename python node file
* Publish temperature array
* Add publisher for PTAT
* Take into account cases when commands are failing
* Refactor colormap loading
* Add Evo Thermal driver
* Contributors: Baptiste Potier, Pierre-Louis Kabaradjian

1.3.0 (2018-10-08)
------------------
* Update maintainer list
* Update Readme for Evo 64px
* Add driver node for Evo 64px
* Contributors: Baptiste Potier, Pierre-Louis Kabaradjian

1.2.1 (2018-09-07)
------------------
* Update Readme for Evo 3m
* Add Evo 3m compatibility to the driver
* Update installation instructions in Readme
* Contributors: Baptiste Potier, Pierre-Louis Kabaradjian

1.2.0 (2018-04-13)
------------------
* Remove unecessary files
* Put the private parameters in define + correct ros info messages
* Update for evo 600Hz
* Contributors: BaptistePotier, Pierre-Louis Kabaradjian

1.1.0 (2017-11-17)
------------------
* Change license to MIT
* Put frame_id as a parameter in one, duo, and evo driver
* Update README with correct evo links
* Contributors: Pierre-Louis Kabaradjian, Baptiste Potier

1.0.1 (2017-09-20)
------------------
* Update package.xml
* Contributors: Pierre-Louis Kabaradjian

1.0.0 (2017-09-15)
------------------
* Create README.md
* Handle infinite values
* Add One and Duo support
* Fix frame dropping on 'T'
* Clean files and change to pragma once
* Implement REP 117 for teraranger duo
* Implement REP 117 for teraranger one
* Use helper lib and clean header of one
* Use helper lib and clean header of duo
* Remove old erial library files
* Update drivers to ros-serial
* Enable outdoor mode in reconfigure for one and duo
* Initialize repository
* Contributors: Pierre-Louis Kabaradjian, BaptistePotier, Mateusz Sadowski
