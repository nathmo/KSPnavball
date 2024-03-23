# KSPnavball
This is a mechanical navball for a custom KSP gamepad
![image](https://github.com/nathmo/KSPnavball/assets/15912256/69ea6f02-c501-4f50-b61d-37bcfbf82153)
![image](https://github.com/nathmo/KSPnavball/assets/15912256/d7e6c4aa-0ebb-4162-b684-228de8cad1dc)

the current status :
![image](https://github.com/nathmo/KSPnavball/assets/15912256/280c12f4-b4f9-488a-9c26-14fd3736ca9e)

# Hardware
|QTY|Name|Link|total price|
|---|----|----|----|
|25X|M3 5 mm depth 4.5mm diamater heat insert |https://www.aliexpress.com/item/1005003582355741.html|3.5$|
|25X|M3 screw | ||
|3X |omniwheel |https://www.aliexpress.com/item/32960657744.html| 8$|
|3X |DC motor |https://www.aliexpress.com/item/33022320164.html|  7$|
|2X |Hbridge DRV8833 |https://www.aliexpress.com/item/1005006444609771.html|      2.5$|
|1X |3 axis magnetometer ||  4.5$ |
|1X |magnet cylinder 10x2mm |https://www.aliexpress.com/item/1005006362930902.html| 2$ |
|1X |arduino nano |https://www.aliexpress.com/item/1005005702204423.html|2.5$|
Total = ~30 $ + 3D printed part + transparent window

one laser cutted sheet of 5mm transparent material is required. It should be engraved with the center sign of the navball.
the plastic part should not require any support and can be printed on a 15 x 15 [cm] bed
you will find the required file to print in the folder ToPrint, same for the folder toCut

# Wiring
need to document

# Software
kinematic
![image](https://github.com/nathmo/KSPnavball/assets/15912256/9c50a9f6-25f3-4878-91e4-69ba9fc491e5)
The sphere is 90 mm in diameter
the wheel are 50 mm in diameter
the wheel are 120° apart (see from the top) and 45° raised from the horizontal plane

https://kerbalsimpitrevamped-arduino.readthedocs.io/en/latest/payloadstructs.html#_CPPv421vesselPointingMessage

https://kerbalsimpitrevamped-arduino.readthedocs.io/en/latest/payloadstructs.html#structvessel_pointing_message
https://www.instructables.com/KerbalController-a-Custom-Control-Panel-for-Rocket/
