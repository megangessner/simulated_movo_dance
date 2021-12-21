# Making Movo Dance

*Final project for CS2951X: Reintegrating AI
Fall 2021*

## Who are you? What is this?
Hi there! I'm Megan Gessner, and this here is the very last thing I do for my Masters degree at Brown. I've been studying CS here for over 5 years now, mostly visual computing and AI, while balancing my passion for dance and choreography. It's been quite the journey trying to reconcile these interests, and I've made a few cool projects towards this end (3D convolutional neural net for transcribing tap dance, a fake Just Dance game with the Kinect, playing with motion capture systems, evaluating dance visualization software), but this particular project emerged in response to the development of the new Choreorobotics 0101 class (offered Spring 2022).

I thought, "oh cool, robots dancing, let's go, I'll just simulate them in Unity, I got my one semester of robotics under my belt, no problem. Once I have it simulated I can use all my AI/machine learning training to do super neat computational creative stuff". Yeah nope, it was extremely difficult and I totally failed, and felt deeply discouraged. I had these big dreams to co-choreograph with a robotic agent and have improvisation sessions where we interacted and learned from each other, but that all was entirely inaccessible so long as the robot was stationary. 

So I vowed that I wouldn't give up, and used this class as an opportunity to take a step forward in this process. In order to eventually get to all the cool stuff I wanted to do that felt dancerly and artistic, I needed to trudge through a whole lot of challenges that were totally outside of my wheelhouse: familiarizing myself with an overwhelmingly large software stack for the MOVO robot, becoming more fluent in ROS and control theory so as to understand what the heck was going on, wrestling with virtual machines and package management and environment configuration, not to mention resolving buttloads of errors that didn't make any sense... it was all painful. **But I did it.** And you can too! That's why I'm writing this documentation of my code, my process, my insights, and my thoughts about future work

My output is far from So You Think You Can Dance quality dancing, but it meets the goal and gets the job done (and it took me a lot of work to get there). It is a simulation of a robot that can dance *on beat* with arbitrary movements. 

## Resources and Literature

### Resources
In this section, I will highlight some of the tools, concepts, and software you will be working with and some resources to learn more:

- We will be using a virtual machine to run Ubuntu *inside* of our native OS. This is kind of bonkers so here's a link if you want to know more. It's basically running a computer inside your computer, but the inner computer doesn't know it's in a simulation. https://www.citrix.com/solutions/vdi-and-daas/what-is-a-virtual-machine.html 
- Sometimes you will be compiling code from source and other times you will be using apt/apt-get to get a software package binary directly. It might be useful to you to review how to compile from source (configure, make, make install), and also what the difference between these two methods are. https://unix.stackexchange.com/questions/152346/what-is-the-difference-between-building-from-source-and-using-an-install-package. 
- ROS, or Robotic Operating System, is a big component of this project. If you have never used ROS before, follow these tutorials: http://wiki.ros.org/ROS/Tutorials to learn some of the basics. As an overview, ROS allows us to create systems of nodes that talk and listen to each other, each with a specific purpose. These nodes are organized into packages, and learning to make these packages within a workspace is a whole other beast: http://wiki.ros.org/catkin/Tutorials. At the heart of ROS is roscore: it contains parameter server with globally-accessible parameters and it makes the connections between nodes, making sure that if one node is interested in what another node has to say, it gets those messages. You can write source code to create these nodes in python or in cpp, and you can interface with the network of nodes and the topics they are about from the command line! Be patient with yourself, go through these concepts multiple times if you need to! I know I did :) I spent some time "echo"-ing the data being sent on various topics of demos, "list"ing nodes and topics, and visualizing the connections of the whole network using rqt_graph (which was helpful for my understanding of the MOVO stack). One cool thing about ROS is it has a bunch of useful packages for various tasks and really excellent documentation of all its packages.
- Moveit https://moveit.ros.org/ is a motion planning framework. It sort of provides a level of abstraction from sending low level commands to actuators on the robot, so that we don't have to worry about such things. We simply think about move groups (parts of the robot that should coordinate together in a motion plan), plans, goals, states, and trajectories. Moveit will work with the controllers (i.e. the actual hardware and low-level software) of the robot and motion planning algorithms like OMPL to actually accomplish whatever needs to get done. We'll be using this a lot, take a look at the tutorials!
- Gazebo and RViz. Gazebo is a simulation software and RViz is a robotics visualization software... their outputs look largely similar but the main difference is that gazebo is trying to simulate the physics and constraints of the real world while RViz is trying to visualize the transforms and the state of the robot, and give us a GUI for planning with moveit. http://gazebosim.org/, http://wiki.ros.org/rviz 

 ### Literature
 In this section 

## Installing the code
This repo contains only my original scripts that belong to the movo_dance package, but in order to get things working, you will need to install many other things.

### OS
The MOVO robot only works on Ubuntu 16.04. If you are like me, you don't have Ubuntu. The department machines have Ubuntu, so you could go try it out there (in fact, I played around on the development PC for MOVO located on the 8th floor of the SciLi to learn how to talk to the actual robot... reach out to Suzanne Alden for card access and Ben Abbattamateo for help). But in this COVID era, your best bet is to develop locally. 
- Install VMWare Workstation Player 16 (https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html) 
- Download a disk image for Ubuntu 16.04 
- Create a virtual machine with this disk image using the VMWare setup wizard
- Finally, open your newly created virtual machine!

#### Snags


