# Realsense Calibration

Realsense SDK offers a calibration data. I think these data should be extremely accurate. (for an example, I have compared the color point cloud build with the official instrinic parameters and the calibration I made with opencv, The official one greatly out-performs the other.)

```
rs-enumerate-devices -c
```

So I choose to take the camera intrinsic parameters, and all the extrinsic parameters.

**IMU calibration**, while this does not offer a ideal IMU calibration. All the parameters are set to identity or zero. And I observe the raw norm of the accelerator (in static) is about 9.0, which is far from the accurate gravity constant ~9.81. As a result, an calibration of the IMU is important.

we can use the [official IMU calibration guide](https://www.intelrealsense.com/wp-content/uploads/2019/07/Intel_RealSense_Depth_D435i_IMU_Calibration.pdf) or use [My Allan Variance Calibration](https://github.com/gggliuye/VINS_PI/tree/main/IMU). After the calibration, the normal of accelerator in static state, get corrected to about 9.806.

**Remark** : the original code for calibration needs a extermely hard and patient process. But we can also change the parameter in line 100 :

```python
self.max_norm = np.linalg.norm(np.array([0.5, 0.5, 0.5]))
``` 

To loose the threshold, for an example to [0.55, 0.55, 0.55], to get a rather good, and much easier calibration.

