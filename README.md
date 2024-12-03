# Pew pew 
WIP!
This is the repository for a turret that fires nerf balls made with [goBILDA](https://www.gobilda.com/) components. All code is written in C++ for arduino. 

## Hardware (The physical thing)
The turrent was comprised of 4 subsystems:
### Drive
🦼The turrent used tank drive and was comprised of two [312 RPM Motors](https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/), each individually controlled by a [1x15A Motor Controller](https://www.gobilda.com/1x15a-motor-controller/).
These controllers were plugged into the PWM ports on an Arduino Giga R1 Wifi, and were powered by a 12V Battery using XT30 connectors. 
### 2 Axis Control
The Turrent was able to Pan and Tilt using two [300° Torque Servos](https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque/). These Servos were both connected to an Arduino Giga R1 Wifi, and allowed nice Rotation and tilt to manage the firing angle of the turret.
### Firing Mechanism
The Firing mechanism is comprised of 2 seperate subsystems: 
#### Rotary Shooter
The rotary shooter is a gear box driven by two [6000 RPM Motors](https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/), and the output of those two are combined into one axel.
That axel is then geared to a 20:100 (1:5) gear ratio, providing an extremely high output velocity.
#### Ammo feeding
The ammo feeder subsystem consisted of a [Speed Servo](https://www.gobilda.com/2000-series-dual-mode-servo-25-3-speed/) and a belt, bringing the victem bullets into the mouth of the rotating wheels. 
### Communications
The Turrent Relies on a [10ch FlySky i6](https://www.amazon.ca/HAWKS-WORK-Transmitter-i6X-iA10B/dp/B0BSBVSVQC/ref=sr_1_1_sspa?crid=3NJ6FR37UMS33&dib=eyJ2IjoiMSJ9.RE0-WjvtVpeM48YKpHwJyoJ_DB5lFDAuPuCYm9DgSnrid7zUALpkUPUYu--tJ30PckSY5I_iGwhlkKmcBatymi6OHemny4cnkLi6MR41g8BeM4ml9hn4Py5GIfFrvty2wZOFHNRf-QbUHs5cIJRmRXOpmDeGwxTOooP7kyZUZGnq866tR_a_prhXrcyskk4kS8lkDMtvm5UFT5y6i9RiWoys4QUh-b2v6kJpwB9cqCEKHW6YYuSYpG5QW9P3LXEQA6advClz-eKth05UFUJdf1zA-ZNNOpCAmoCjqaqdy64.65HPUWjF5wBxTaTVraNa4DDNZsxzFRUux4Q4yrdcmJI&dib_tag=se&keywords=flysky+10ch&qid=1733197176&sprefix=flysky+10ch%2Caps%2C213&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) 
that's inputs are processed by an Arduino Mega using the IbusBM librairy. 

## Software 
The Turret uses 2 c++ scripts and one custom lib to run.
