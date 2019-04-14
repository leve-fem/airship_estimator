# airship_estimator

This package was developed for comparative porposes. It contains algorithms for pose and velocity estimation of a robotic airship. The algorithms available are: Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF) and Low-pass Filter (LPF).

Results obtained are available in:
[LPF](youtu.be/VL5dvCyOZwY)
[EKF](youtu.be/jaATwV0rG30)
[UKF](youtu.be/B26xaKtAyWo)

## First steps

1. Clone this project.

2. Install [robot_localization](http://wiki.ros.org/robot_localization) 

3. Download the file [data_set.bag](https://www.dropbox.com/s/abjkcnbxy7qy39h/data_set.bag?dl=0) and save inside the folder bag_files

4. Run command below for running the data set available
```
roslaunch airship_estimator play_data_set.launch
```
## How to use

You can configure UKF and EKF parameters by editing the file:
```
config/ukf_params.yaml
```
You can configure LPF parameters by editing the file:
```
launch/lpf.launch
```
You can configure Acceleration correction parameters by editing the file:
```
config/accel_estimator_params.yaml
```

In files ```launch/play_data_set.launch``` and ```launch/play_results.launch``` you can comment/uncomment some lines to change between EKF/UKF/LPF filters as you want.

Before running ```launch/play_results.launch``` you will have to change the name of the bag file inside the launch with the one generated by ```launch/play_data_set.launch```.
