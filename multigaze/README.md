MultiGaze
=========

Tunnel:
------

Connect just 1 camera to the PC and neglect the other 5.

```
  make clean
  make TUNNEL=1
  make upload2   (= MultiGaze only)
```

 or

```
 ./tunnel.sh 1
```

Normal:
------

Uploads all 7 stm32f4 with code: 6x stereo with their calibration parameters and 1x multigaze project to merge.

```
  make clean
  make upload   ( all 7 STM32's)
```

  - unplug black-magic-pro
  - plug while pressing button
  - make -C blackmagic
