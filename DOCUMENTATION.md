# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


[image1]: ./output_images/1.png "Output Image 1"
[image2]: ./output_images/2.png "Output Image 2"
[image3]: ./output_images/3.png "Output Image 3"
[image4]: ./output_images/4.png "Output Image 4"


# Model Predictive Controller Project

##  Intro

Details for Udacity SDCND MPC Project. Goals:

- To navigate a track in a Udacity-provided simulator, which communicates telemetry & track waypoint data via websocket, by sending steering and acceleration commands back to the simulator.

- Solution robust to 100ms latency, as one may encounter in real-world application.

## OUTPUT IMAGES

![alt text][image4]

![alt text][image2]

![alt text][image3]

![alt text][image1]

It makes use of the **ipplot** and **cppad** libraries to calculate :

- an optimal trajectory & its associated actuation commands in order to minimize error with a third-degree polynomial fit to the given waypoints.
- The optimization considers only a short duration's worth of waypoints, and produces a trajectory for that duration based upon a model of the vehicle's kinematics and a cost function based mostly on the vehicle's cross-track error (roughly the distance from the track waypoints) and orientation angle error, with other cost factors included to improve performance.

## Rubric Points

### - The Model:

The kinematic model includes the :
- vehicle's __x__ and __y__ coordinates,
- vehicle's orientation angle __(psi)__, and
- vehicle's __velocity__, as well as
- the cross-track error and psi error __(epsi)__.

**Actuator** outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the below equations:

```
xt+1 = xt + vt * cos(Ψt) * dt
yt+1 = yt + vt * sin(Ψt) * dt
Ψt+1 = Ψt + vt/Lf * ẟt * dt
vt+1 = vt + at + dt
ctet+1 = f(xt) -yt + (vt * sin(eΨt) * dt)
eΨt+1 = Ψt - Ψdest + (vt/Lf * ẟt * dt)
```

### - Timestep Length and Elapsed Duration (N & dt):

The values taken for **N** and **dt** are **10** and **0.1**, respectively after trying other set of values like (20 / 0.06, 8 / 0.127, 6 / 0.15), which were erratic in behavior. These values mean that the optimizer is considering a **1 sec** duration in which to determine a corrective trajectory. Adjusting either **N** or **dt** (even by minute amounts) often produced erratic behavior.

### Polynomial Fitting and MPC Preprocessing:

The **waypoints** are pre processed by transforming them to the vehicle's perspective (in main.cpp line 108). It simplifies the code to fit a polynomial to the waypoints because the vehicle's **x** and **y** coordinates are now at the :
- **origin (0, 0)** and,
- the **orientation angle is also 0**.

## Model Predictive Control with Latency:

Mechanism to deal with latency is done in 2 ways other than simply limiting the speed:

- The original kinematic equations depend upon the actuations from the previous time step, but with a delay of **100ms** (which happens to be the time step interval) the actuations are applied with another time step later, so the equations have been altered to account for this (**MPC.cpp** lines 104-107).

- In addition to the cost functions suggested in the course, (punishing CTE, epsi, difference between velocity and a reference velocity, delta, acceleration, change in delta, and change in acceleration) an additional cost penalizing the combination of velocity and delta (**MPC.cpp** line 63) was coded to get controlled cornering.
