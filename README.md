# roboy_matrix
The following instructions will lead you through the setup process for all the modules deployed on the Roboy's Raspberry Pi.

## Pre-requisites
1. Get the image with Rasbian from [here](https://drive.google.com/open?id=1AAVTsoHEOwQvT1hpvIBMKmyn1j7TyBZE) 
```
user: pi
password: raspberry
```
2. Install ROS Kinetic. Follow installation instructions from the [ROS Wiki](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)

## Pi Setup
Currently we are using [Matrix Creator kernel modules](https://github.com/matrix-io/matrixio-kernel-modules) to read/wirte sensor data from the board. The modules are kernel drivers and therefore one can get the data directly usring `regmap` or `libiio`. 
1. In case you used a fresh image of Raspbian, follow [these instructions](https://github.com/matrix-io/matrixio-kernel-modules/blob/master/README.md). Otherwise, if the image mentioned in pre-requised was used:
```
sudo apt update
sudo apt upgrade
cd ~/matrix-kernel-modules
git pull origin master
cd src
make && make install
sudo reboot
```
2. Add matrix kernel modules to `/etc/modules`, which should contain:
```
matrixio-regmap
matrixio-everloop
matrixio-gpio
matrixio-imu
matrixio-env
```
3. Add overlay:
```
echo "dtoverlay=matrixio" | sudo tee -a /boot/config.txt
```

## Test
- Check if Matrix Creator kernel modules are enabled:
```
ls -l /dev/matrix_*
```
The output should include `matrix_everloop`and `matrix_regmap`.
- Record an audio sample from the microphone array:
```
cd ~/
arecord --duration 5 --rate 16000 --format S16_LE test.wav && aplay ./test.wav
```
