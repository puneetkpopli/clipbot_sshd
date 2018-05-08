#include <libssh/libssh.h>
#include <libssh/server.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <iostream>

#include <nlohmann/json.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using json = nlohmann::json;
using namespace std;

#define equals(X, Y) (strcmp(X, Y) == 0)

bool connectionAcceptedAndKeysExchanged(ssh_session &session, ssh_bind &bindingOpts) {

  session = ssh_new();
    
  if (ssh_bind_accept(bindingOpts, session) == SSH_ERROR) {
      
    printf("error accepting a connection : %s\n", ssh_get_error(bindingOpts));
    return false;

  } else if (ssh_handle_key_exchange(session)) {
      
    printf("ssh_handle_key_exchange: %s\n", ssh_get_error(session));
    return false;
  }

  return true;
}

bool userAuthenticated(ssh_session &session) {

  ssh_message message;
  bool isAuthenticated = false;
    
  do {
    if (!(message = ssh_message_get(session)))
      break;

    if (ssh_message_type(message) == SSH_REQUEST_AUTH) {
      if (ssh_message_subtype(message) == SSH_AUTH_METHOD_PASSWORD) {

	if (equals(ssh_message_auth_user(message), "user")
	    && equals(ssh_message_auth_password(message), "pass")) {

	  isAuthenticated = true;
	  ssh_message_auth_reply_success(message, 0);
	} else {
	  ssh_message_auth_set_methods(message, SSH_AUTH_METHOD_PASSWORD);
	  ssh_message_reply_default(message);
	}
      } else {
	ssh_message_auth_set_methods(message, SSH_AUTH_METHOD_PASSWORD);
	ssh_message_reply_default(message);
      }
    } else {
      ssh_message_reply_default(message);
    }  
    ssh_message_free(message);
  } while (!isAuthenticated);
    
  if (!isAuthenticated) {
    printf("auth error: %s\n",ssh_get_error(session));
    ssh_disconnect(session);
    return false;
  }
  return true;
}

ssh_channel openChannel(ssh_session &session) {

  ssh_message message;
  ssh_channel channel = 0;

  do { 
    if (message = ssh_message_get(session)) {
      if (ssh_message_type(message) == SSH_REQUEST_CHANNEL_OPEN) { 
	if (ssh_message_subtype(message) == SSH_CHANNEL_SESSION)
	  channel = ssh_message_channel_request_open_reply_accept(message);

      } else {
	ssh_message_reply_default(message);
      }
      ssh_message_free(message);
    }	
  } while (message && !channel);
    
  if (!channel) {
    printf("error : %s\n", ssh_get_error(session));
    ssh_finalize();
  }
  return channel;
}

bool isShellChannel(ssh_session &session) {

  ssh_message message;
  bool isShellRequest = false;
    
  do {
    message = ssh_message_get(session);
    
    if (message
	&& ssh_message_type(message) == SSH_REQUEST_CHANNEL
	&& ssh_message_subtype(message) == SSH_CHANNEL_REQUEST_SHELL) {
      
      isShellRequest = true;
      ssh_message_channel_request_reply_success(message);
      break;
    }
    
    if (!isShellRequest)
      ssh_message_reply_default(message);
    
    ssh_message_free(message);    
  } while (message && !isShellRequest);
  
  if (!isShellRequest) {
    printf("error : %s\n",ssh_get_error(session));
    return false;
  }
  
  return true;
}

int main(int argc, char **argv) {
  
  ssh_message message;
  ssh_session session;   
  ssh_bind bindingOpts = ssh_bind_new();

  ros::init(argc, argv, "clipbot_sshd_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("twist", 1000);

  ssh_bind_options_set(bindingOpts, SSH_BIND_OPTIONS_DSAKEY, "/home/user/src/sshd/keys/ssh_host_dsa_key");
  ssh_bind_options_set(bindingOpts, SSH_BIND_OPTIONS_RSAKEY, "/home/user/src/sshd/keys/ssh_host_rsa_key");
  ssh_bind_options_set(bindingOpts, SSH_BIND_OPTIONS_BINDPORT_STR, "7005");

  if (ssh_bind_listen(bindingOpts) < 0) {
    printf("Error listening to socket: %s\n", ssh_get_error(bindingOpts));
    return 1;
  }
  
  ssh_channel channel;
  char buf[256];
  int i;  

  while (true) {
    
    if (!connectionAcceptedAndKeysExchanged(session, bindingOpts))
      return 1;
    
    if (!userAuthenticated(session))
      return 1;
    
    if (!(channel = openChannel(session)))
      return 1;

    if (!isShellChannel(session))
      return 1;
    
    do {

      if (!ros::ok())
	break;
      
      i = ssh_channel_read(channel, buf, 256, 0);
      if (i > 0) {
	buf[i] = 0;
	json j;
	try {
	  
	  j = json::parse(buf);	  
	  //cout << j["x"].get<float>() << " " << j["y"].get<float>() << endl;
	  geometry_msgs::Twist twist;
	  twist.linear.x = j["y"].get<float>();
	  twist.angular.z = j["x"].get<float>();
	  pub.publish(twist);
	  
	} catch (json::parse_error &e) {}
      }
    } while (i > 0);

    ssh_disconnect(session);
  }
  
  ssh_bind_free(bindingOpts);
  ssh_finalize();
  return 0;
}
