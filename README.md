# robot_sshd
This ROS project accepts connections from BotCtrl and forwards joystick input to an Arduino as Twist messages w/rosserial.

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
