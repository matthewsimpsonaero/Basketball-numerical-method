# Basketball Numerical Method

<p align="center">
    <img src="https://github.com/matthewsimpsonaero/Basketball-numerical-method/blob/main/Final.gif" alt="Alt Text" width="400" height="300"/>
</p>



This code was written as an exta credit project for MAE 361 - Vibrations taught by Dr. Larry Silverberg
[MAE 361 - Vibrations taught by Dr. Larry Silverberg](https://www.mae.ncsu.edu/people/lmsilver/). For this activity, a State Space Analysis was utilized to compute the physical motion of a basketball interacting with an enviroment, complete with a ball launcher, horizontal and verical walls, and an event triggering basketball hoop.

Derived equations of motion were integrated using a second order [Runge–Kutta](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods) numerical method. The basic outline of the code is as follows.

<ins>Equations of motion:</ins>

``` math
\ddot{x} = \frac{\mu N}{m} //
```
``` math
\ddot{y} = (N/m) - g //
```
``` math
\ddot{\theta} = \frac{-2F}{mr}//
```

<ins>Secord order Runge-Kutta integration:</ins>

$$ x(t + \Delta T) = x(t) + f \Delta T \left[ \frac{\partial f}{\partial x} f + \frac{\partial f}{\partial t}\right] \frac{\Delta T^2}{2}  = \alpha f + \beta \left[  f + \frac{\partial f}{\partial x} f + \frac{\partial f}{\partial t} \Delta T\right]$$

where 

$$ \Delta T = \alpha + \beta     $$        $$ \beta = \frac{\Delta T}{2}$$