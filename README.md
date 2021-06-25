# RISE Laboratory Knee Exoskeleton
This repository is responsible for the code base implemented onto the RISE lab knee exoskeleton. Below you will find guides for using the knee exoskeleton, more specifically how to start the exoskeleton and how to trouble shoot the exoskeleton.

## Getting Started
To begin, you need to change the directory to where the GUI is located and run the code below.

```
$ python3 GUI.py
```
Afterwards you will see the GUI show up on your window. Note: it will take a couple of seconds to load.

(Pictures will be included)

Now before we use the exoskeleton, we need to calibrate it. Clicking the "calibrate" button will attempt to do this. During the calibration sequency, the GUI sends a calibration command to the odrive through the Odrive Python API. More information is available on the Odrive Robotics website. https://docs.odriverobotics.com/ 

During calibration, you should hear a noticeable beep and then the knee joint will in one direction then swap directions and finish. If the exoskeleton does not swap directions, there is something wrong. See the troubleshooting section. 

After finishing the calibration phase, you can begin to use the exoskeleton. 

If there is a noticeable offset, you can calibrate the sensors by click on their respective buttons.

## Control System
The controller has two components, high-level control and low-level control. The high-level control currently has two modes, "Zero-impedance" and "Sinusoidal." The zero-impedance mode essentially sets the reference to zero which is equivalent to making the knee join experience virtually zero impedance as it moves. The sinusoidal mode sets the reference to a sinusoidal response. You can adjust the sine wave's frequency and amplitude in the GUI. 

## Software
The custom software is embedded into two primary components, the CPU (Raspberry Pi 4) and the microcontroller (Teensy 4.1). The CPU is dedicated to running the GUI, which is used to read data, while the microcontroller is dedicated to running the control algorithms and reading the sensor data. The two components communicate over Serial USB so it sending data to the CPU from the microcontroller does not cause a large delay to the CPU. It should be noted that besides to initialization phase, the microcontroller does not require any information from the CPU while it is activating the exoskeleton.

## Simulation

## Troubleshooting

### GUI says "High-level control broke"

This issue is mostlikely due to a communication issue between either the microcontroller or odrivetool. First check if they're properly connected. You can also check using the commandline. Run the command below.

```
$ lsusb
```

Make sure you see something related to the odrive and microcontroller, if you do not then it means that the CPU is not detecting them.

Another reason this could be happening is if the microcontroller, is not properly communicating with the CPU. First try unplugging the microcontroller and plugging it back in.If it still does not work, try to check the data the microcontroller is sending to the CPU. You can do this two ways, the first is using Putty and the second is running the sensorReading script.

(What the expected data looks like is coming soon)

```
$ python3 sensorReading.py
```

### The exoskeleton is not calibrating

This can be due too several issues with the hardware and software. To begin, please check that the odrive is properly connected to the exoskeleton; more specificaly to the motor encoder and the motor itself. 

(Pictures will be included)

Next, run the command to open the odrivetool.

```
$ odrivetool
```

Now, check that the odrive is able to see the encoder count. 

```
<axis>.encoder.shadow_count
```

If the odrive is not able to read the encoder counts, then that should be the issue that is preventing the exoskeleton from calibrating. First, check the wires that are connected to the motor encoder with a continuity test. Try to do this multiple times, adjusting the wire positions. 

If the connector is working then there is only two more things to test, if the issue is with the Odrive or if the issue is with the encoder. You can check either one, to check the encoder just get an oscilloscope check the pulses on every pin. To check the odrive, get another encoder and see if the odrive can read that encoder with the shadow_count command.

