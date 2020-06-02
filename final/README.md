# Multirobot-system
Robotics system to control operations to transport an object through a maze, like a robot might do in a factory setting. From its initial location, the Create2 will carry an object through a maze to a robotic arm, which will pick up the object and place it on a shelf specified by the user. The code needs is easily extendable to support slight changes in the environment like different mazes or changes in the coordinate system.

## Install V-Rep

Go to  http://www.coppeliarobotics.com/downloads.html and download V-REP  Pro Edu V3.5.0  (rev4) for your platform.

    Windows
Follow the installer to install V-REP. There will be a shortcut on your desktop to run it after the installation is complete.

    Mac OS X
Launch the vrep executable after downloading the archive. You might need to change your security settings to allow applications from an untrusted source.

    Linux
You can download and extract the tar.gz file in any folder. To launch V-REP, open a terminal, navigate to the extracted V-REP folder, and execute
[ yourusername ]$ ./ vrep . sh

## Install Python3

    Linux (Ubuntu)
On a command prompt use:
[ yourusername ]$ sudo apt - get install python 3 python3 - numpy python3 - matplotlib

    Windows
Download the python3 Anaconda distribution  32-bit  from  https://www.anaconda.com/download/  —  We tested Anaconda 5.2.0 with python 3.6. Please make sure you install the 32-bit version for Windows as there are some issues with the 64-bit version installation for working with V-REP. To make things easier, select “Add Anaconda to my PATH environment variable” as this will make Anaconda’s python3 environment the default on your system. Please note that you will be able to run python by using python rather than python3 as on Mac OS X and Ubuntu.
Verify your installation by opening a terminal (e.g. PowerShell) and run “python”. A text beginning with “Python 3.6.5 |Anaconda, Inc.|. . . ” should appear.
If you get the error ’python’ is not recognized as an internal or external command, operable program or batch file., please follow these steps:
1.    Press Windows + R and run sysdm.cpl.
2.    Switch to the “Advanced” tab and click on “Environment Variables. . . ”’.
3.    Under System variables, select the line that begins with “Path” and click on “Edit. . . ”.
4.    Click on “New” and type in the path to your Anaconda installation. It will by default be in your home directory, i.e., C:\Users\<USER NAME>\Anaconda3.
5.    Click on “Ok” in this window and in the previous window.
6.    Open a new terminal and run “python” to verify the Anaconda environment setup.

    Mac OS X
Download Anaconda (Python 3.5) from https://www.anaconda.com/download/ and follow the installer. You can verify your installation by running python3 --version in a Terminal.



## Run the application

1.    In V-REP, open the Lab1.ttt scene file by using the “File/Open scene. . . ” menu. You should be able to see a virtual iRobot Create2 with a cup on top of it, a maze, a maniupulator robot and a shelf.
2.    In a terminal with the extracted zip-file or source code as your working directory:
(On Windows use python rather than python3.)
[ yourusername ]$ python3 run.py --sim finalproject.py
3.    In V-REP, you should see start seeing the robot turn around to localize itself and it knows where it is it will start moving and try to reach the arm robot.
