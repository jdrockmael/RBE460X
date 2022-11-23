# RBE460X
Surface Layer value in the Gazebo GUI controls friction

## Joy-Con drivers
https://github.com/nicman23/dkms-hid-nintendo

https://github.com/DanielOgorchock/joycond

# Packages

## joycon
### controller.py 
will cause rumble in the joycon 

Subscriber= leftlevelSetter, rightlevelSetter

Publisher= None

### joyStickReader.py 
will publish the reading from the joycon sticks asynchronously

Subscriber= None

Publisher= leftStickPos, rightStickPos

## interface_controller_bot
### robotDrive.py
drive the robot based on the joystick values

Subscriber= leftStickPos, rightStickPos

Publisher= /cmd_vel
