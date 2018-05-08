# robot_sshd
This ROS project is a simple ssh daemon adapted from the sshd example bundled with libssh. It accepts connections from my <a href="https://github.com/static55/BotCtrl">BotCtrl</a> Android app and forwards the joystick input it receives to an Arduino as Twist messages via rosserial. The Arduino code will be made available shortly. Check <a href="https://github.com/static55">here</a>.

# Installation

#### Install libssh if you don't already have it:

`sudo apt-get install libssh-4`<br>
`sudo apt-get install libssh-dev`

#### Clone repo

`cd ~/catkin_ws/src`<br>
`git clone https://github.com/static55/robot_sshd.git`

#### Create sshd keys somewhere

`user@clipbot:~/catkin_ws/src/$ cd ~`<br>
`user@clipbot:~$ mkdir robot_sshd_keys`<br>
`user@clipbot:~$ cd robot_sshd_keys`<br>
`user@clipbot:~/robot_sshd_keys$ ssh-keygen -t DSA`<br>

```
Generating public/private DSA key pair.
Enter file in which to save the key (/home/user/.ssh/id_dsa): ./ssh_host_dsa_key
Enter passphrase (empty for no passphrase): <Hit return>
Enter same passphrase again: <Hit return>
Your identification has been saved in ./ssh_host_dsa_key.
Your public key has been saved in ./ssh_host_dsa_key.pub.
```

`user@clipbot:~/robot_sshd_keys$ ssh-keygen -t RSA`<br>

```
Generating public/private RSA key pair.
Enter file in which to save the key (/home/user/.ssh/id_rsa): ./ssh_host_rsa_key
Enter passphrase (empty for no passphrase): <Hit return>
Enter same passphrase again: <Hit return>
Your identification has been saved in ./ssh_host_rsa_key.
Your public key has been saved in ./ssh_host_rsa_key.pub.
```

#### Update ~/catkin_ws/src/robot_sshd/src/robot_sshd_node.cpp for your setup

I'll get around to adding support for command line parameters eventually :P

* Change the hardcoded username 'user' and password 'pass' in `userAuthenticated()`
* Update the `ssh_bind_options_set()` calls with the paths to your keys

#### Build and run

`cd ~/catkin_ws`<br>
`catkin_make`<br>
`catkin_make install`<br>
`roslaunch robot_sshd robot_sshd.launch`

#### Start BotCtrl on your phone and connect
