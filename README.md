## Search and Rescue **Based on Colias**
It is a simulation implemented on the Player/Stage, which presents the swarm intelligence used in the scene of searching and rescuing based on the robot Colias.

#### Usage 
```
./sar_colias.sh number [size_x*size_y]*
```
**number** robots will be generated randomly in the area of **size_x * size_y**. If the parameter **number** is not specified, last config will be adopted. Generally, the parameter **size** is given, there is no need to provide it if using the same map.

#### Vagrant
To run the program in other platforms, we provide [Vagrant](https://www.vagrantup.com/) to build the virtual environment. You need to install Vagrant and VirtualBox (or other virtual machine), and run:
```
vagrant up
```
to boot up the virtual machine, then:
```
vagrant ssh -- -X xterm
```
to connect to the vm with graphic outputs allowed.