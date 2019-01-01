# Control and Estimation of a 3D Quadrotor

The goal here is to develop the estimation portion of a Quadrotor controller. Specific instructions can be found in the [README.md](./README.md) in the same directory as this file.

## Step 1: Sensor Noise ##

Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

For this step, choose the scenario `06_NoisySensors`. This scenario creates the tabular data files `config/log/Graph1.txt` (GPS X data) and `config/log/Graph1.txt` (Accelerometer X data). The quickest way to get standard deviation tabular data in a file is to use Pandas and the `dataframe.describe()` method.  The following code was run in an ipython shell. 
```
In [1]: import pandas as pd                                                                                        

In [2]: a = pd.read_csv('config/log/Graph1.txt')                                                                   

In [3]: a.describe()                                                                                               
Out[3]: 
            time   Quad.GPS.X
count  99.000000    99.000000
mean    5.004892     0.000583
std     2.872242     0.712774
min     0.105000    -2.137262
25%     2.554997    -0.460496
50%     5.004820     0.039863
75%     7.454643     0.508141
max     9.905373     1.467805

In [4]: b = pd.read_csv('config/log/Graph2.txt')                                                                   

In [5]: b.describe()                                                                                               
Out[5]: 
              time   Quad.IMU.AX
count  1992.000000   1992.000000
mean      4.982393     -0.015919
std       2.875887      0.487506
min       0.005000     -1.854940
25%       2.493752     -0.340058
50%       4.982322     -0.013225
75%       7.470891      0.301203
max       9.960395      1.973880
```

Next, we plug in these results into the top of `config/6_Sensornoise.txt`.  Now, rerun the simulator to check that the measurements are within +/- 1 sigma.

Result
```$xslt
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 71% of the time
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 68% of the time
```

## Step 2: Attitude Estimation ##

Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.

For this scenario, choose `Scenario -> 7_AttitudeEstimation` from the simulator menu.  

To stabalize the motion, we implement 3 functions in `QuadEstimatorEKF.cpp`:
 - `UpdateFromIMU()`

Performance Evaluation
 - Your attitude estimator needs to get within 0.1 rad for each of the Euler angles for at least 3 seconds.

Result
```
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
```

In the screenshot below the attitude estimation using a linear scheme (left) and using the improved non-linear scheme (right).  Note the reduction in the Y axis on the right.

![attitude example](images/attitude-screenshot.png)

## Step 3: Prediction Step ##

Next, implement the prediction step of the filter.  For this step, use `Scenario -> 8_PredictState`.  This will create a quad that files a box pattern.

In `QuadEstimatorEKF.cpp` implement the state prediction step in the `PredictState()` function.  A correct implementation will have a slow drift as shown below.

![predict drift](images/predict-slow-drift.png)

Now run `09_PredictionCov` and tune the `QPosXYStd` and the `QVelXYStd` process parameters in `QuadEstimatorEKF.txt` to capture the magnitude of the error. The resulting values are,

```
QPosXYStd = .03
QVelXYStd = .2
```

With the above parameters the simulations should look like this:

![good covariance](images/predict-good-cov.png)

## Step 4: Magnetometer Update ##

Now we implement the magnetometer update. For this step, use `Scenario -> 10_MagUpdate`.  In this scenario, the estimated yaw is drifting away from the tru yaw.  We need to implement a better yaw estimator.

First, tune 1 parameter in `QuadEstimatorEKF.txt`:
 - `QYawStd` adjust to approximately capture the magnitude of the drift.
 
Reasonable parameter for `QYawStd` was found to be:
```$xslt
QYawStd = .2
```

Next, implement the magnetometer update in the function `UpdateFromMag()`.  

Performance Evaluation
 - have an estimated standard deviation that accurately captures the error
 - maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation

Result
```
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 76% of the time
```

With the above parameters the simulations should look like this:

![mag good](images/mag-good-solution.png)

## Step 5: Closed Loop + GPS Update ##

Now we run scenario `11_GPSUpdate`.  We set `Quad.UseIdealEstimator` to 0 in `config/11_GPSUpdate.txt` and comment out the following lines:
```$xslt
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```

Next, we tune the remaining parameters in the process noise model.
```$xslt
QPosZStd = .05
QVelZStd = .2
```

Finally, implement the EKF GPS Update in the function `UpdateFromGPS()`.

Performance Evaluation
 - complete the entire simulation cycle with estimated position error of < 1m

Result
```
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```

## Step 6: Adding Your Controller ##

Note: I used my original `QuadController.cpp` and `QuadControlParams.txt` for the entire project.  The default solution code for `AltitudeControl()` and `LateralPositionControl()` was incorrect. 

Result
```
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```