#!/usr/bin/env python

import rospy
from edo_core_msgs.srv import *
from datetime import datetime
import os
import base64

class SystemCmdNode():
    def __init__(self):
        rospy.init_node('system_cmd_node')
        s = rospy.Service('system_command_srv', SystemCommandSrv, self.handle_system_mdd_srv)
        print "Ready"
        rospy.spin()

    def handle_system_mdd_srv(self, req):
        print req, req.command
        if req.command == 0:
            ts_ms = datetime.utcnow().strftime("%s") + '000' # non sono sicuro
            return SystemCommandSrvResponse('{}'.format(ts_ms))
        elif req.command == 1:
            ip, net = req.data.split()
            ff = "#!/bin/bash\n\nsudo ifconfig eth0 {} netmask {}".format(ip, net)
            with open("/home/edo/configLAN", "w") as f:
                f.write(ff)
            os.system("chmod 755 /home/edo/configLAN")
            os.system("/home/edo/configLAN")
            return SystemCommandSrvResponse('')
        elif req.command == 2:
            f = os.popen('ifconfig eth0 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
            ip=f.read().strip()
            f = os.popen('ifconfig eth0 | grep "inet\ addr" | cut -d: -f4 | cut -d" " -f1')
            net=f.read().strip()
            return SystemCommandSrvResponse('{} {}'.format(ip, net))
        elif req.command == 3:
            os.system("sudo kill -9 $(ps aux | grep 'create_ap -d -n -w2' | awk '{print $2}')")
            data = req.data.split()
            if len(data) == 2:
                s, p = data[0], data[1]
            else:
                s, p = data[0], ""
            ssid, password = base64.b64decode(s), base64.b64decode(p)
            if 8 <= len(password) <= 63 or len(password) == 0:
                ff = '#!/bin/bash\nsudo kill -9 $(ps aux | grep \'create_ap -d -n -w2\' | awk \'{{print $2}}\')\nsudo /usr/bin/create_ap -d -n -w2 --no-virt wlan0 {} {} &'.format(ssid, password)
                with open("/home/edo/configWifi", "w") as f:
                    f.write(ff)
                os.system("chmod 755 /home/edo/configWifi")
                os.system("/home/edo/configWifi")
                return SystemCommandSrvResponse('')
            else:
                return SystemCommandSrvResponse('Error')
        elif req.command == 4:
            f = os.popen('ps aux | sed -n \'/.*[c]reate.*--no-virt wlan0 /{s///p;q}\'')
            data=f.read().strip().split()
            ssid = base64.b64encode(data[0])
            psk = ''
            if len(data) == 2:
                psk = base64.b64encode(data[1])
            return SystemCommandSrvResponse('{} {}'.format(ssid,psk))
        return SystemCommandSrvResponse('not implemented')


if __name__ == '__main__':
    node = SystemCmdNode()
