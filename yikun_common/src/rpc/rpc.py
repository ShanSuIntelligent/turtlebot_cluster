#!/usr/bin/env python
from yikun_cluster_msgs.srv import *
import rospy
import socket

_clients=["192.168.99.61","192.168.99.62","192.168.99.63","192.168.99.64","192.168.99.65"]
_port=20001

def send_rpc(cmd):
    if cmd == 'all':
        for ip in _clients:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            server_address = (ip,_port)
            try:
                sock.connect(server_address)
                sock.sendall(cmd)
                sock.close()
                print ("send to " + ip + " " + cmd)
            except socket.error:
                print("connect to " + ip + " error")
    else:
        index = int(cmd)-1
        ip = _clients[index]
        server_address = (ip, _port)
        print("send to " + ip + " " + cmd)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)

        try:
            sock.connect(server_address)
            sock.sendall(cmd)
            sock.close()

        except socket.error:
            print("connect to "+ip+" error")
def update():
    rospy.wait_for_service('set_path')
    try:
        rpc_real = rospy.ServiceProxy('update_path', Rpc)
        resp1 = rpc_real('start')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def rpc_callback(req):
    print req.cmd
    update()
    send_rpc(req.cmd)
    return RpcResponse(req.cmd)

def rpc_server():
    rospy.init_node('rpcNode')
    rpc = rospy.Service('rpc', Rpc, rpc_callback)
    
    rospy.spin()

if __name__ == "__main__":
    rpc_server()
