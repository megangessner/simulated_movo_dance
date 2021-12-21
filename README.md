# Making Movo Dance

*Final project for CS2951X: Reintegrating AI
Fall 2021*

## Who are you? What is this?
Hi there! I'm Megan Gessner, and this here is the very last thing I do for my Masters degree at Brown. I've been studying CS here for over 5 years now, mostly visual computing and AI, while balancing my passion for dance and choreography. It's been quite the journey trying to reconcile these interests, and I've made a few cool projects towards this end (3D convolutional neural net for transcribing tap dance, a fake Just Dance game with the Kinect, playing with motion capture systems, evaluating dance visualization software), but this particular project emerged in response to the development of the new Choreorobotics 0101 class (offered Spring 2022).

I thought, "oh cool, robots dancing, let's go, I'll just simulate them in Unity, I got my one semester of robotics under my belt, no problem. Once I have it simulated I can use all my AI/machine learning training to do super neat computational creative stuff". Yeah nope, it was extremely difficult and I totally failed, and felt deeply discouraged. I had these big dreams to co-choreograph with a robotic agent and have improvisation sessions where we interacted and learned from each other, but that all was entirely inaccessible so long as the robot was stationary. 

So I vowed that I wouldn't give up, and used this class as an opportunity to take a tiny but significant step forward to unblock myself in this process. In order to eventually get to all the cool stuff I wanted to do that felt dancerly and artistic, I needed to trudge through a whole lot of challenges that were totally outside of my wheelhouse: familiarizing myself with an overwhelmingly large software stack for the MOVO robot, becoming more fluent in ROS and control theory so as to understand what the heck was going on, wrestling with virtual machines and package management and environment configuration, not to mention resolving buttloads of errors that didn't make any sense... it was all painful. **But I did it.** And you can too! That's why I'm writing this documentation of my code, my process, my insights, and my thoughts about future work

This is a simulated robot dancing pre-defined but totally arbitrary movements to the beat of music. My output is far from ready to take to the Broadway stage, but it meets my goal and gets the job done (and it took me a lot of work to get there). 

## Resources and Literature

### Resources
In this section, I will highlight some of the tools, concepts, and software you will be working with and some resources to learn more:

- We will be using a virtual machine to run Ubuntu *inside* of our native OS. This is kind of bonkers so here's a link if you want to know more. It's basically running a computer inside your computer, but the inner computer doesn't know it's in a simulation. https://www.citrix.com/solutions/vdi-and-daas/what-is-a-virtual-machine.html 
- Sometimes you will be compiling code from source and other times you will be using apt/apt-get to get a software package binary directly. It might be useful to you to review how to compile from source (configure, make, make install), and also what the difference between these two methods are. https://unix.stackexchange.com/questions/152346/what-is-the-difference-between-building-from-source-and-using-an-install-package. 
- Github. We will be using git to get code from various places, so familiarize yourself with the CLI (git clone, git pull, etc.). 
- ROS, or Robotic Operating System, is a big component of this project. If you have never used ROS before, follow these tutorials: http://wiki.ros.org/ROS/Tutorials to learn some of the basics. As an overview, ROS allows us to create systems of nodes that talk and listen to each other, each with a specific purpose. These nodes are organized into packages, and learning to make these packages within a workspace is a whole other beast: http://wiki.ros.org/catkin/Tutorials. At the heart of ROS is roscore: it contains parameter server with globally-accessible parameters and it makes the connections between nodes, making sure that if one node is interested in what another node has to say, it gets those messages. You can write source code to create these nodes in python or in cpp, and you can interface with the network of nodes and the topics they are about from the command line! Be patient with yourself, go through these concepts multiple times if you need to! I know I did :) I spent some time "echo"-ing the data being sent on various topics of demos, "list"ing nodes and topics, and visualizing the connections of the whole network using rqt_graph (which was helpful for my understanding of the MOVO stack). One cool thing about ROS is it has a bunch of useful packages for various tasks and really excellent documentation of all its packages.
- Moveit https://moveit.ros.org/ is a motion planning framework. It sort of provides a level of abstraction from sending low level commands to actuators on the robot, so that we don't have to worry about such things. We simply think about move groups (parts of the robot that should coordinate together in a motion plan), plans, goals, states, and trajectories. Moveit will work with the controllers (i.e. the actual hardware and low-level software) of the robot and motion planning algorithms like OMPL to actually accomplish whatever needs to get done. We'll be using this a lot, take a look at the tutorials! If you want to dig deeper, 
- Gazebo and RViz. Gazebo is a simulation software and RViz is a robotics visualization software... their outputs look largely similar but the main difference is that gazebo is trying to simulate the physics and constraints of the real world while RViz is trying to visualize the transforms and the state of the robot, and give us a GUI for planning with moveit. http://gazebosim.org/, http://wiki.ros.org/rviz 
- Audio signals! In order to understand beat detection algorithms, it's a good idea to get a sense for how audio data is represented digitally so that we may sample and analyze it for semantic information: https://developer.mozilla.org/en-US/docs/Web/Media/Formats/Audio_concepts

 ### Literature
In this section I will share links to some of the literature that I consumed to inspire and actualize this project. These approaches point towards some of possible next steps for this project, including movement generation, stylization, interactivity, music synchronization, using dance frameworks, encoding/representation, learning from demonstration and more!

#### The State of Choreorobotics and Bringing Choreographic/Embodied Practices to Coding and Robotics
- https://www.nytimes.com/2020/11/05/arts/dance/dance-and-artificial-intelligence.html
- https://www.researchgate.net/publication/322048839_Choreographic_and_Somatic_Approaches_for_the_Development_of_Expressive_Robotic_Systems
- https://scholar.google.com/citations?user=qw0ukB4AAAAJ&hl=en
- https://scholar.google.com/citations?hl=en&user=u5_b1QYAAAAJ

#### Real-time Beat Tracking
- http://c4dm.eecs.qmul.ac.uk/papers/2009/StarkDaviesPlumbley09-dafx.pdf
- Librosa (library for music analysis and feature extraction, MFCC's are used in the above paper) https://librosa.org/doc/main/generated/librosa.feature.mfcc.html

#### Synchronizing Motion with Real-time Beat Tracking
- https://www.researchgate.net/publication/220851546_Dance_Motion_Control_of_a_Humanoid_Robot_Based_on_Real-Time_Tempo_Tracking_from_Musical_Audio_Signals
- https://ieeexplore.ieee.org/document/4399244/authors#authors
- https://eita-nakamura.github.io/articles/Ohkita_etal_DancingRobot_2017.pdf (This one is performing with a human partner, using a similar beat tracking algorithm but with visual input)
- https://core.ac.uk/download/pdf/288499803.pdf (This one is very detailed as it is a thesis and has a great over view of various beat-tracking algorithms and their pros and cons. The system description is actually very similar to what I ended up creating)

#### Learning from Demonstration/ Other interfaces
- https://github.com/tsitsimis/dmpling/blob/master/literature/ijspeert-NC2013.pdf (dmp letter)
- https://spectrum.ieee.org/how-to-make-a-robot-dance

#### Computational Creativity (in generating movements to dance to music)
-  Feel The Music: Automatically Generating A Dance For An Input Song - https://arxiv.org/abs/2006.11905 (not robots)
-  https://arxiv.org/pdf/1911.02001.pdf
- https://www.researchgate.net/publication/316945707_Learning_by_Demonstration_for_a_Dancing_Robot_within_a_Computational_Creativity_Framework (This one also has learning from demonstration in it, but not with DMPs, that's unique to my approach)
- https://github.com/NVlabs/Dancing2Music

#### Interactivity
- https://expressivemachinery.gatech.edu/projects/luminai/

#### Human-Robot Motion Retargeting
- https://ieeexplore.ieee.org/document/7989632

#### Miscellaneous (check back for updates)
Here is a link to my google sheet containing cool Choreorobotics-y papers. I have a local repo of papers that I am continuing to add to google drive. 
https://docs.google.com/spreadsheets/d/1ZQjM19S0j9qf0zXMlPpSTpdnFSfhZxAlr_g6eIJTXpE/edit#gid=0

## Installing the code
This repo contains only my original scripts that belong to the movo_dance package, but in order to get things working, you will need to install many other things.

### OS
The MOVO robot only works on Ubuntu 16.04. If you are like me, you don't have Ubuntu. The department machines have Ubuntu, so you could go try it out there (in fact, I played around on the development PC for MOVO located on the 8th floor of the SciLi to learn how to talk to the actual robot... reach out to Suzanne Alden for card access and Ben Abbattamateo for help). But in this COVID era, your best bet is to develop locally. 
- Install VMWare Workstation Player 16 (https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html) 
- Download a disk image for Ubuntu 16.04 
- Create a virtual machine with this disk image using the VMWare setup wizard
- Finally, open your newly created virtual machine!

### MOVO stack and ROS
The robot used in this project is Kinova's MOVO. Brown has one on the 8th floor of the SciLi. I chose this robot because it's got 7dof, which is kind of exciting from a choreographic perspective (it can get into all sorts of weird configurations that our human bodies can't do!). 
- Clone the kinova-movo git repo: https://github.com/Kinovarobotics/kinova-movo/, then checkout the kinetic-devel branch.
- Follow the Setup Instructions https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions, replacing any place you see "indigo" with "kinetic". (Kinetic and Indigo are two different ROS distributions). A couple of the packages in the "additional packages" section will say they can't be found, that's okay, just remove those from the installation command. 
- As part of these instructions, you will create a catkin workspace and you will install ROS. Remember to source your devel/setup.sh and the ros installation's setup file in your ~/.bashrc. (The instructions will guide you through this, and refer to the ROS tutorials above if you need to).

At this point you will notice that the MOVO stack has **A LOT** of files, and not a whole lot of documentation. Not to worry, I will walk you through it! Well, I'll at least walk you through the relevant ones, if I don't mention it it's probably not particularly useful to you. Each of the folders denotes a catkin package (some of them contain multiple packages). Remember that a package implies a sort of semantic grouping of functionality, specifying nodes, messages, services, and scripts. 
- `kinova_api`: this is a deb distribution referring to some of the kinova source code from here[https://github.com/Kinovarobotics/kinova-ros/wiki] to move arms. You don't have to worry about it, but you can poke into the repo if you are interested. 
- `movo_7dof_moveit_config` and `movo_moveit_config`: MOVO robots can either have 7dof JACO arms or 6dof JACO arms. These moveit packages get the robot configured to be used by Moveit given the particular kinematics of the arms you want to use. For the simulation, we're going to use 6dof (movo_moveit_config).
- `movo_common`: very important set of packages! Contains movo-specific msgs (`movo_msgs`), example python scripts and utilities for executing movements (`si_utils`), a URDF description of the movo with mesh files (`movo_description`) (very important for talking about the structure and specifications of the robot), 
ros interfaces to joint controllers and actuators as well as action clients (hey do this!) (`movo_ros`), and more!
- `movo_demos` : contains demos. This package took me forever to figure out. Most of the demos are specified as launch files, so you can go into `\launch` to see the various demos you can launch. Many of these launch files launch a bunch of nodes from various packages in order in order to spin up controllers, sensors, and visualization. Some of them refer to a joystick or sensors... you can ignore/comment out those nodes, those are for teleoperating the real robot. If you want to get a simple demo going, try to `roslaunch movo_demos sim_demo_show_basic.launch`. This will bring up some fake controllers, the moveit_commander interface, an rviz window and a gazebo window
- `movo_simulation`: gazebo-related plugins that get called by `movo_demos`. You will see worlds, scripts, models, controllers, and launch files that simulate the real-world robot. 
- `movo_robot`: this code runs on the actual robot, it's sort of the core of the robot. 

In my dance repo, you will see a `movo_dance_zone.launch` and a `dance_zone.sdf`. These are simple scenes for gazebo so the simulation doesn't have to render complex environments we aren't interacting with. Move `movo_dance_zone.launch` into `movo_simulation/movo_gazebo/launch` and move `dance_zone.sdf` into `movo_simulation/movo_gazebo/worlds`. These two files are referenced by my movo_dance.launch code. Good practice for your linux command skills ;) 

### DMP
Recall we are using dynamic motion primitives as our basis for constructing arbitrary, time-parameterized, generalized motions. Clone the ros dmp package from github[https://github.com/sniekum/dmp] into your movo_ws/src directory. This package gets called on by my scripts in this repo. 

### simulated_movo_dance
This repo!! Clone it in your `movo_ws/src` directory. A walkthrough of the code is here, with a voiceover of my design decisions and how to use the scripts:

[Click to watch demo part 1 here](https://drive.google.com/file/d/1Ti4_bHnetZilJbNc1pfgGvobXB7Cde78/view?usp=sharing)
[Click to watch demo part 2 here](https://drive.google.com/file/d/1ymyAAYiPOmSGNXkUGbnBMQC2OaFwADIw/view?usp=sharing)
[Click to watch demo part 3 here](https://drive.google.com/file/d/1MZrfApPEMGDvxcgESDMhujMUPBeh9Kn1/view?usp=sharing)

(Sorry this is split into multiple parts... I used a free trial of a screen recorder)

### BTrack and libsamplerate
BTrack is a library for doing real-time beat tracking using onset detection and hypothesis evaluation: https://github.com/adamstark/BTrack It's written in cpp but it has a python module which I call on in my code. 
In order to use BTrack, you must first compile and install libsamplerate, found here: https://github.com/libsndfile/libsamplerate . Call ./autogen.sh, followed by ./configure, followed by make, and finally make install. 
Clone both of these repos inside of this one (`simulated_movo_dance`). To install the python module for BTrack, follow the install README instructions in the python-module folder. 

#### Snags
- For some reason VMWare and Gazebo don't vibe particularly well together. In order to maximize performance, we are going to want to enable 3D graphics acceleration in our virtual machine. To do so, make sure to install VMWare Tools. Then, before launching the virtual machine, click the player settings, under display there is a checkbox for 3D graphics acceleration. If you can't click that checkbox, you will need to install a different version of VMWare tools. There is a default installation in the Ubuntu image but you will want to install a different distribution (https://kb.vmware.com/s/article/1022525). 
- Then, if you try to run gazebo and get an ioctl error, follow this suggestion: https://robocademy.com/2020/05/02/solved-opengl-issues-with-gazebo-and-vmware/, which we do to downgrade our opengl version.  
- Also verify that your version of gazebo (once it is installed) is gazebo7. If it is not, follow instructions online to change the version number. 
- If you ever get a permission denied error, use sudo. 
- If a package is missing, try to sudo apt-get install it! Or perhaps pip install if it is a python import error. 

## Future Work
So what is next? Firstly, it is rather inconvenient to program movements for the robot in a sort of hardcoded way, commanding it to go to a particular joint state, validate that it is valid, store it, generate a DMP for a given pose list (that I have to manually specify) etc. Instead, I would like to use a Kinect sensor (which the movo has mounted on its body) or other motion-tracking system in order to extract movements and store the trajectories. Then, these trajectories would be mapped onto the MOVO's kinematics via a visual shape/structure reconstructive loss minimization algorithm. These could then be ported to DMPs and stored in the movement vocabulary for future vocabulary. Being able to teach the MOVO how to move by demonstration would be much more supportive of choreographers and non-coders and much faster in terms of iteration. The next, mildly obvious, step is to go from simulation to real world. This is a matter of making sure the scripts are portable and interfacing with the robot's controllers whether the MOVO is running in simulation or for real. I was hoping to get to these two features in this iteration of the project, but alas. 

With this, I would like to teach the robot a model for producing movement based on music, other dancers, and general embodied knowledge about the state space of dance movements and the relations between them. To accomplish this, I have considered using William Forsythe's improvisation technologies, as well as Laban Notation, to encode features of movement beyond just "here's a bunch of poses, do them robot", because that doesn't get at the artistry and meaning-making inherent in improvisation and performance. These frameworks could be used to extract and/or create meaningful patterns in movement, which in turn could lead to some sort of Hidden Markov Model where next movements are created based on past movements, observations of other dancers in the space, observations of musical features (genre, timbre, beat, rhythm), to create a transition matrix. 

But this challenge itself poses another challenge: collecting movement data, annotating it with semantic information (its relationships to other known movement patterns, like noticing that a particular movement emerges in the improvisation patterns of a particular group of people with similar training), discovering latent structure (shapes, symbols, imagery), trying to determine emotion (which would require tagging the movement with the dancer's internal sensation and experience of the movement), and so on. This requires efficient compression of movement, retrieval of movement from databases, comparison and analysis of movement, and a way of collecting labelled data in this way from objective shapes to subjective expressions. Algorithms are beginning to emerge for these problems but it's still an open research question I hope to work on. 

I can also think of several cool applications of this code that can be used in the future. For example, I imagine a jam session between between robot and dancer where the partners respond to each other via various relations (mirroring, transposing, repetition, adversarial games, etc.). This can be grounded in exercises with reward functions (standards of success in the exercise) which the robot can then learn to become a better partner improvisationalist. I can also think of an application where I might want to choreograph a piece, and get stuck at a particular movement, and then ask the robot to give me a cool idea. The robot might know my tendencies and suggest something that is in my style, or it may suggest something that is totally tangential to my style, adding a new flavor to my choreography and perhaps opening me up to new creative possibilities. Sort of like a collaborate assistant! This has been done to some degree in other contexts (will be added to the choreorobotics lit review soon), but not so much with robotics, which requires so much more mathematics and control theory. 
