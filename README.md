# clipbot_sshd
This ROS project is a simple ssh daemon adapted from the sshd example bundled with libssh. It accepts connections from my <a href="https://github.com/static55/BotCtrl">BotCtrl</a> Android app and forwards the joystick input it receives to an Arduino as Twist messages via rosserial. The Arduino code is <a href="https://github.com/static55/clipbot_arduino">here</a>.

# Installation

#### Install libssh and rosserial if you don't already have them:

`sudo apt-get install libssh-4`<br>
`sudo apt-get install libssh-dev`<br>
`sudo apt-get install ros-kinetic-rosserial` (not needed to compile but is run in the launch file)

#### Clone repo

`cd ~/catkin_ws/src`<br>
`git clone https://github.com/static55/clipbot_sshd.git`

#### Create sshd keys somewhere

`user@clipbot:~/catkin_ws/src/$ cd ~`<br>
`user@clipbot:~$ mkdir clipbot_sshd_keys`<br>
`user@clipbot:~$ cd clipbot_sshd_keys`<br>
`user@clipbot:~/clipbot_sshd_keys$ ssh-keygen -t DSA`<br>

```
Generating public/private DSA key pair.
Enter file in which to save the key (/home/user/.ssh/id_dsa): ./ssh_host_dsa_key
Enter passphrase (empty for no passphrase): <Hit return>
Enter same passphrase again: <Hit return>
Your identification has been saved in ./ssh_host_dsa_key.
Your public key has been saved in ./ssh_host_dsa_key.pub.
```

`user@clipbot:~/clipbot_sshd_keys$ ssh-keygen -t RSA`<br>

```
Generating public/private RSA key pair.
Enter file in which to save the key (/home/user/.ssh/id_rsa): ./ssh_host_rsa_key
Enter passphrase (empty for no passphrase): <Hit return>
Enter same passphrase again: <Hit return>
Your identification has been saved in ./ssh_host_rsa_key.
Your public key has been saved in ./ssh_host_rsa_key.pub.
```

#### Update ~/catkin_ws/src/clipbot_sshd/src/clipbot_sshd_node.cpp for your setup

* Change the hardcoded username 'user' and password 'pass' in `userAuthenticated()`
* Update the `ssh_bind_options_set()` calls with the paths to your keys

#### Build and run

`cd ~/catkin_ws`<br>
`catkin_make`<br>
`catkin_make install`<br>
`roslaunch clipbot_sshd clipbot_sshd.launch`

#### Start BotCtrl on your phone and connect

# License

Public Domain. Do whatever you want with it.
