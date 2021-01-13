# Polhemus Viper ROS driver

This driver as a part of the bachelor project "Recreating Operator Paths Using an Electromagnetic Motion Tracker and a Robot Manipulator" by Gabriel Gosden for Technological Insitute, DMRI.

This ROS driver is custom built for the Polhemus Viper electromagnetic motion tracker. It it made using the Libusb 1.0 API.

# Dependencies 

This ROS driver depends on the UR devopled ROS driver: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver but can with minor modification to the launch file be used independently.

# Install 

To allow the Viper driver to run as non-root it is needed to modify the .rules file. This is done by using:

```
$ cd ./etc
$ ./install_rules.bash
```

Note that you will require the `sudo` password. 

# How to change the Viper settings

Change Viper settings in the `config/viper_settings.yaml` file. The following settings are implemented to be changed using the launch file. 
- Hemisphere
- FTT mode
- Position units
- Orientation units
- Sensor filter level

The rest of the Viper settings are default.




