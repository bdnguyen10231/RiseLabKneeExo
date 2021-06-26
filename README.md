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

(Pictures will be included)

During calibration, you should hear a noticeable beep and then the knee joint will in one direction then swap directions and finish. If the exoskeleton does not swap directions, there is something wrong. See the troubleshooting section. 

After finishing the calibration phase, you can begin to use the exoskeleton. 

If there is a noticeable offset, you can calibrate the sensors by click on their respective buttons.

(Pictures will be included)

## Control System
The controller has two components, high-level control and low-level control. The high-level control currently has two modes, "Zero-impedance" and "Sinusoidal." The zero-impedance mode essentially sets the reference to zero which is equivalent to making the knee join experience virtually zero impedance as it moves. The sinusoidal mode sets the reference to a sinusoidal response. You can adjust the sine wave's frequency and amplitude in the GUI. 

The figure below shows how the control system is structured. First, the high level controller determines the reference torque and that is sent to the low level controller which is then used to compute the desired current to get the exoskeleton output to match the reference.

![ExoControl](https://user-images.githubusercontent.com/57163503/123485489-13a54480-d5bf-11eb-8377-378493529515.png)

### High-Level Controller

As mentioned earlier, the high level controller has two modes, however, more modes can be added. This section is aimed at showing you how. Below is an example of how a sinusoidal reference should be implemented on the microcontroller.

The way to implement a high level controller is to first create a int variable that will only be 0 or 1, indicating whether or not the controller is on or off. In the hlc.h header file write something like this.

```
class hlc {

public:
  int sinusoidal = 0;

  float freq = 0.0;
  float amp = 0.0;

  float reference = 0.0;

  float controller(); 

};
```

Now in the source file write something like this

```
float hlc::controller() {

  if (sinusoidal == 1) {
    reference = amp*sin(freq*current_time*2*M_PI/1000);
  }

  return reference = 0;
}
```

In the main loop, the high-level controller will return a reference every iteration. Remember to add a method in the GUI in order to activate the mode and send it over to the microcontroller. Once the microcontroller controller recieves the message, it will attempt parse the data based on the commas. In the main arduino script, there is a function called "getPiData", this is the function that parses the data from the GUI. In order to read an additional value in the microcontroller, we have to parse the data manually. Below is an example of how the data with two commas(three values) is parsed.

```
void getPiData(String message) {

  // gets the location of the comma's
  int commaIndex = message.indexOf(',');
  int secondCommaIndex = message.indexOf(',', commaIndex + 1);
  
  // gets values between the comma's
  String firstValue = message.substring(0, commaIndex);
  String secondValue = message.substring(commaIndex + 1, secondCommaIndex);
  String thirdValue = message.substring(secondCommaIndex + 1);
  
  // cast the String data to its correct type
  float data = firstValue.toFloat();
  float mode1 = secondValue.toInt();
  float mode2 = thirdValue.toInt();
  
}
```

### Low-Level Controller

The Low-Level Controller uses a PID control structure. The derivativation for the discretization is performed by [Scilab](https://www.scilab.org/discrete-time-pid-controller-implementation). The low pass filter is tuned by collecting data and using a matlab code to see how the roll-off frequency affects the output. The code is included inside the "simulation" folder.

To tune the PID controller, use the GUI.

(Pictures will be included)

## Other Software (ODrive)

Using the odrive independently of the GUI and microcontroller is crucial at times, especially for debugging. You can find all this information on the odrive [documentation](https://docs.odriverobotics.com/) but I will go through an example with on how to run the odrive for basic torque control.

To begin, we need to open the odrivetool so go into a terminal and type:

```
$ odrivetool
```

Afterwards, wait for the odrive to connect. Then to manually calibrate the motor use this command:

```
<axis>.requested_state = 3
```

Where the number 3 is the key to the full calibration sequence, as documented by the odrive [docs](https://docs.odriverobotics.com/api/odrive.axis.axisstate). It should be noted that axis is where the motor is connected. Typically it is odrv0.axis0 for our setup.

Now to send a torque command, first we need to make sure sure the odrive is in close loop control mode, which is state = 8.

```
odrv0.axis0.requested_state = 8
```

Make sure nothing that is connected to the odrive is sending torque commands, for example the microcontroller. Then use this command to input the reference torque:

```
odrv0.axis0.motor.config.torque_constant = 0
```

Setting the reference to zero causes is essential zero-impedence mode.

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

