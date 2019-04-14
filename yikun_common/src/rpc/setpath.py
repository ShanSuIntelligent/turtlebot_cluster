#!/usr/bin/env python
import sys
import rospy
from yikun_cluster_msgs.srv import *
import socket

def triger_signal(cmd):
    rospy.wait_for_service('set_path')
    try:
        rpc_real = rospy.ServiceProxy('set_path', Rpc)
        resp1 = rpc_real('start')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def start_server():
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server_address = ("0.0.0.0",20001)
    sock.bind(server_address)
    sock.settimeout(1.0)
    sock.listen(5)
 
    while True:
        print "waiting .........."
        connetion,client_address = sock.accept()
        try:
            print  "Connection from ",client_address
            cmd = connetion.recv(1024)
            print "cmd: '%s'" % cmd
            triger_signal(cmd)
        finally:
            connetion.close()


print start_server()

