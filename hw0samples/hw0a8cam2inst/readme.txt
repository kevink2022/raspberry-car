
Instructions to set up the camera and install RaspiCAM library for 
  camera projects:

[1] Turn off your Raspberry Pi computer, and 
      attach a camera to Raspberry Pi computer.  Turn on your Raspberry Pi computer.

[2] Enable the camera by running "sudo raspi-config" and selecting 
      "Interfacing Options" --> "Legacy Camera" and enable the camera.
      Then reboot your Raspberry Pi computer.

[3] Start a terminal, type "sudo apt-get install cmake" command. (You need internet
      connection to your Raspberry Pi computer.)

[4] Download hw0a8cam2inst.tgz file from the CMPEN 473 Homework 0 page.
      Untar the file.

[5] Go to the hw0a8cam2inst project directory, type "cd raspicam-0.1.9" command 
      to go into the raspicam-0.1.9 directory.

[6] Type "cmake CMakeLists.txt" command in the raspicam-0.1.9 directory.

[7] Then type "make" command in the raspicam-0.1.9 directory.  
      You may ignore any warning signals.

[8] Then type "sudo make install" command in the raspicam-0.1.9 directory.

[9] Then type "sudo ldconfig" command in the raspicam-0.1.9 directory.
      Now the RaspiCAM library is installed.  Run a simple program to 
      check the camera, continue follow the instruction below.

[10] Now cd back to the hw0a8cam2inst project directory, and 
      type "make" command.

[11] Type "sudo ./hw0a8cam2inst" to take a picture with camera.

[12] See the two picture files, you may use Image Viewer.  One picture is 
       will be in full color and the other picture is black and white (gray).
