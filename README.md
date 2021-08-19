# VINS-RAW

[video bilibili](https://www.bilibili.com/video/BV1yK411G75Z/)

[video youtube](https://www.youtube.com/watch?v=-7wahX6hKLk&t=31s)

### Dependences：

1. pangolin: <https://github.com/stevenlovegrove/Pangolin>
2. opencv
3. Eigen
4. Ceres: use ceres for the initialization part

### Build

```c++
mkdir build
cd build
cmake ..
make -j
```

### Run

#### 1. CurveFitting Example to Verify Our Solver.
```c++
cd build
../bin/testCurveFitting
```

#### 2. VINs-Mono on Euroc Dataset
```c++
cd build
../bin/run_euroc
```

### Development

#### 2020/11/12 :
successfully run in a calibrated RealSense D435i camera (Mono + IMU mode)

**TODOs**:
* update the solver for a faster process.
  - Keep the jacobian unchanged if the parameters not changed too much.
  - Try to use a different optimizer. (DogLeg or PCG)
  - Build faster the Hessian matrix.
  - Try sparse Hessien matrix.
* update the fault detector of my solver.
