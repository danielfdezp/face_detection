# face_detection
In this repository you will find the necessary code to run the facedetect model from Xilinx and rotate a camera when a person is detected.
This file uses the /dev/mem tool to map the necessary components and use them.
In this case, I had to map the Axi Timer addresses so I could generate a PWM signal through the PMOD module.
