# KSPnavball
This is a mechanical navball for a custom KSP gamepad
![image](https://github.com/nathmo/KSPnavball/assets/15912256/69ea6f02-c501-4f50-b61d-37bcfbf82153)
![image](https://github.com/nathmo/KSPnavball/assets/15912256/d7e6c4aa-0ebb-4162-b684-228de8cad1dc)

# Current state
currently it can be assembled but the wheel have nylon wheel and are too slippery. I'm experimenting with wheel made out of TPU but I need something even more rubbery and sticky. 
Regarding the software the main idea is here and the ball move but due to the slippage it's hard to tell if the code is doing it's job or if I fucked up the math to get the transform. (plus it definitely need some tunning)
anyways, feel free to build it and help me iron out the kink :)
![image](https://github.com/nathmo/KSPnavball/assets/15912256/280c12f4-b4f9-488a-9c26-14fd3736ca9e)

# open issue
Ball slippage, omniwheel grip issue. I need to find a way to make the wheel more sticky and ruberry so that it works correctly.

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

https://kerbalsimpitrevamped-arduino.readthedocs.io/en/latest/payloadstructs.html#_CPPv421vesselPointingMessage

https://kerbalsimpitrevamped-arduino.readthedocs.io/en/latest/payloadstructs.html#structvessel_pointing_message
https://www.instructables.com/KerbalController-a-Custom-Control-Panel-for-Rocket/

# Kinematic
![image](https://github.com/nathmo/KSPnavball/assets/15912256/9c50a9f6-25f3-4878-91e4-69ba9fc491e5)

The sphere is 90 mm in diameter
the wheel are 50 mm in diameter
the wheel are 120° apart (see from the top) and 45° raised from the horizontal plane

To compute the required motor commande $\alpha_1, \alpha_2, \alpha_3$ we need to break down the problem in two parts : 
1) finding the required rotation to go from the current navball orientation to the target one.
2) project this rotation on the wheels so that we know how much to rotate each motor.

### Find the required rotation
this is solved using [quaternion](https://en.wikipedia.org/wiki/Quaternion). We know two things :
1) the ball current orientation in the form of a unitary quaternion $(c_0, c_1, c_2, c_3)$ and
2) the target orientation in the form of a unitary quaternion $t_0, t_1, t_2, t_3$

from theses we know that there must exist a unitary quaternion such that $q \cdot c = t $ and thus we can deduce $q = tc^{-1}$
$c^{-1}$ being the conjugate of c, compute as $c^{-1}=(c_0, -c_1, -c_2, -c_3)$. unitary mean that the norm $\sqrt{c_0^2+c_1^2+c_2^2+c_3^2}$ of the quaternion is equal to 1.
in practice this is compute like this :

$$
\begin{aligned}
q_0 &: c_0 t_0 + c_1 t_1 + c_2 t_2 + c_3 t_3, \\
q_1 &: -c_1 t_0 + c_0 t_1 - c_3 t_2 + c_2 t_3, \\
q_2 &: -c_2 t_0 + c_3 t_1 + c_0 t_2 - c_1 t_3, \\
q_3 &: -c_3 t_0 - c_2 t_1 + c_1 t_2 + c_0 t_3.
\end{aligned}
$$

$q_0$ represent the ammount of rotation and $q_1, q_2, q_3$ represent the rotation axis direction $(x,y,z)$

### project it on the motor axis
we use linear algebra to project the quaternion axis on the axis of each motor. then we scale this by the required rotation.

first we can precompute the vector basis on which we are going to project the rotation (3 vector that represent the rotation made by the three omniwheel)
we have our motor raised 45° from the horizontal plane. the unit axis for the first motor is thus :

$$
\text{First axis } (u_1): \text{Rotate vertical } (z) \ 45^\circ \text{ around } y:
u_1 = \begin{bmatrix}
\frac{\sqrt{2}}{2} \\
0 \\
\frac{\sqrt{2}}{2}
\end{bmatrix}.
$$

then we apply a $\pm 120°$ rotation around the z axis and obtain the two other vector.

$$
\text{Second axis } (u_2): \text{Rotate } u_1 \ +120^\circ \text{ around } z:
u_2 = \begin{bmatrix}
-\frac{\sqrt{2}}{4} \\
\frac{\sqrt{6}}{4} \\
\frac{\sqrt{2}}{2}
\end{bmatrix}.
$$

$$
\text{Third axis } (u_3): \text{Rotate } u_1 \ -120^\circ \text{ around } z:
u_3 = \begin{bmatrix}
-\frac{\sqrt{2}}{4} \\
-\frac{\sqrt{6}}{4} \\
\frac{\sqrt{2}}{2}
\end{bmatrix}.
$$

we know that there must be a vector $\alpha$ such that \mathbf{M} \alpha = \mathbf{w}, where $\alpha$ are the "ammount" projected on each wheel, $w$ is the rotation axis found previously using quaternion. $M$ is the matrix made from $\mathbf{M} = \begin{bmatrix} \mathbf{u}_1 & \mathbf{u}_2 & \mathbf{u}_3 \end{bmatrix}$

thus if we invert the matrix M we get :
$$
 = \frac{1}{9} \begin{bmatrix}
10 & -2 & -2 \\
-2 & 10 & -2 \\
-2 & -2 & 10
\end{bmatrix}.
$$
this matrix is computed once and stored in the code and the problem can now be easily solved using a matrix product $\alpha = \mathbf{M}^{-1}\mathbf{w}$

Now we convert the Quaternion to Rotation Vector to get w

$$
\text{Given } \mathbf{q} = (q_0, q_1, q_2, q_3),

\begin{aligned}
\text{Rotation angle:} \\
\theta &= 2 \arccos(q_0). \\
\text{Rotation axis (normalized):} \\
\mathbf{v} &= \frac{1}{\sqrt{1 - q_0^2}} \begin{bmatrix} q_1 \\ q_2 \\ q_3 \end{bmatrix}. \\
\text{Rotation vector:} \\
\mathbf{w} &= \theta \mathbf{v}.
\end{aligned}
$$
(we get w by encoding the ammoung of rotatation into v the unitary axis vector.


# Calibration
we have a magnetometer and a magnet embeded in the sphere, how to calibrate and then deduce the sphere orientation from the magnetic field felt by the magnetometer ?



