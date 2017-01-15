# RacingHexapods
Race a hexapod with a Q-learning walking algorithm against one with an uninformed search method of walking.

This was my project for the class, "CS182: Artificial Intelligence" taught by Professor Scott Kuindersma at Harvard.

View CS182FinalReport.pdf for a full report of the project description, methods, and results.

# Instructions for Running Simulation

This will make more sense if you read CS182FinalReport.pdf first, but...

To start, you must download V-REP software: http://www.coppeliarobotics.com/downloads.
html. It is a quick and easy download. Then download the files CS182FinalReport-QLearning.ttt and CS182FinalReport-Race.ttt and Hexapod-Qdata.txt. The first demonstrates a hexapod Qlearn- ing through 500 steps. The second demonstrate a race between a QLearned agent, Greedy Agent, and UCS2Generation agent. You will need to place these files in the scene folder within the VREP files. It will be called something like: V-REP-PRO-EDU-V3-3-2-Mac/scenes.

To start VREP, go to the command line and cd into V-REP-PRO-EDU-V3-3-2-Mac or your com- puter’s equivalent. Then run the command ./vrep.app/Contents/MacOS/vrep if you have a Mac, or the equivalent for the computer you have. This is if you want VREP to print variables to the command line. Otherwise you can just open the VREP application directly.

Once open, go to the file menu and open one of the two scenes that you put in the scene folder. Once you open one of these scenes, you can press play to begin the simulation. You can investigate the code by clicking the document icon in the middle of the left panel. The code is located in the Threaded file, not the main file. Note: If the race simulation of CS182FinalReport-Race.ttt is too slow, you can select the leftmost hexapod (the UCS2Generation agent) and press delete.

To run CS182FinalReport-Race.ttt, you will need the Qvalues for the Qlearning robot. That’s what is in Hexapod-Qdata.txt. You need to download this file and then copy the path to that filecin your computer. Then go to the one line in the code that says ”io.open” and paste your full path to the file (just replace the current file path that is there). Then you can run the race. I already uploaded Qvalues to the text file by training with the LeftRight-Origin gait on 500 steps. If you don’t change the path name, then VREP will freeze.

For CS182FinalReport-QLearning.ttt, you can just press play, and the Q-Learning will start from scratch. I chose the FrontBack-Vertical walking gate for this file, but you can go into the code and change the gait. You can do this by searching for the part of the file with the comment ”CHANGE CONSTANTS HERE”. You only have to change legPairs and the right vertex value. You choose which legs are symmetric to which by matching the index of the array with the value (in LUA index starts at 1). The red foot is 1 and the blue foot is 2. The legs go from 1 to 6. For FrontBack gaits, use right-v-num = 6, for LeftRight gaits, use right vertex = 1. Make sure to rotate either the your perspective or the robot itself so that you see it walking straight depending on which value you chose for the vertices.

In case your computer is not letting you use the .ttt scene files, I have attached the CS182FinalReport- QLearning.ttt code on the course website. To get this code into the simulation, you first have to drag the hexapod model from the V-REP interface. The one I used is located under the Model Browser in robots/mobile. You just drag hexapod1 into the scene. Once it is in the scene, presscthe scripts logo in the middle of the left sidebar. You will need to modify the Threaded script and delete the Non-threaded script. Double click the threaded script and paste in the code. Double click the non-threaded script and delete all the code inside. Then you can press the play button, and it will start QLearning.

