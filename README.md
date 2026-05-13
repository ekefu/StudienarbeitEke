Author: 
Furkan Eke 
https://www.linkedin.com/in/furkaneke
kontakt@neusys3d.de

Here is my 'small thesis' work (German: Studienarbeit) from 2005. 

A copy of the work might be existing in the archive of the Technical University of Munich where I was studying and submitted this work. But it was -to my knowledge- was not publicly available. 

In the original work, I did the following:

- programmed a static human hand kinematic model using Denawit Hartenberg translation matrices,
- programmed a commercial robotic hand kinematic model using Denawit Hartenberg translation matrices,
- programmed a kinematic grasping movement of both hand joints, to generate a dataset of joint angles versus fingertip positions,
- trained feed forward neural networks to learn the backward kinematics of the robotic both hand,
- tried to find an optimized mapping from the fingertip positions of a teleoperator human hand onto the fingertips of the robotic hand for remote grasping tasks.

During the theoretical study, I restricted the programmatic work to a subspace of the robotic hand to avoid kinematic singularity positions, to enable smooth mapping between the two hands.

All program code was written in Matlab scripting language, and the neural network training was done using the Matlab Neural Network Toolbox.

The initial commit contains:
- this readme file 
- the final PDF of the thesis report
- all the source code in its original form, without modifications or improvements.
- trained network models

-o-

As the original code was written in a commercial not free software of Matlab, I intend to take help from coding agents to convert it to either python or julia code and replace Matlab Neural Network Toolbox with scikit-learn.
