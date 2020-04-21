# Software License Agreement (BSD)
#
# @author    Mike O'Driscoll <modriscoll@clearpathrobotics.com>
# @copyright (c) 2017, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import absolute_import, print_function, unicode_literals

import gtk
import rospy
import dbus
import dbus.service
import dbus.mainloop.glib
import bluetooth
import threading

from bluetooth_pair.msg import BluetoothDevice, BluetoothDevices
from bluetooth_pair.srv import PairDevice, PairDeviceResponse, SearchDevices, SearchDevicesResponse

SERVICE_NAME = "org.bluez"
AGENT_IFACE = SERVICE_NAME + '.Agent1'
ADAPTER_IFACE = SERVICE_NAME + ".Adapter1"
DEVICE_IFACE = SERVICE_NAME + ".Device1"

class Agent(dbus.service.Object):
    pass

class BluetoothPairNode(object):
    def __init__(self, node_name='bluetooth_pair'):
        rospy.init_node(node_name)
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

        self.paired = False
        self.pair_error = False
        self.found_devices = dict()
        self.search_filter = ""

        # Setup dbus for interacting with bluez
        self.bus = dbus.SystemBus()
        self.dbus_path = '/ros/bluetooth_pair'
        self.agent = Agent(self.bus, self.dbus_path)
        self.manager = self.proxyobj(self.bus, '/', 'org.freedesktop.DBus.ObjectManager')
        objects = self.manager.GetManagedObjects()
        for path, ifaces in objects.iteritems():
            adapter = ifaces.get(ADAPTER_IFACE)
            if adapter is None:
                continue
            obj = self.bus.get_object('org.bluez', path)
            self.adapter = dbus.Interface(obj, ADAPTER_IFACE)

        # Configure ROS services
        self.discovery_in_progress = False
        self.request_scan_service = rospy.Service('search_devices',
                                                  SearchDevices, self.search_devices)
        self.pair_service = rospy.Service('pair_device',
                                          PairDevice, self.pair_device)
        self.remove_service = rospy.Service('remove_device',
                                            PairDevice, self.remove_device)

        # Configure ROS publishers
        self.list_devices_pub = rospy.Publisher('discovered_devices', BluetoothDevices, queue_size=1, latch=True)
        self.list_paired_pub = rospy.Publisher('paired_devices', BluetoothDevices, queue_size=1, latch=True)

    def proxyobj(self, bus, path, interface):
        # see: https://github.com/Douglas6/blueplayer/blob/master/blueplayer.py
        # see: https://stackoverflow.com/questions/14262315/list-nearby-discoverable-bluetooth-devices-including-already-paired-in-python
        obj = bus.get_object('org.bluez', path)
        return dbus.Interface(obj, interface)

    def filter_by_interface(self, objects, interface_name):
        # see: https://github.com/Douglas6/blueplayer/blob/master/blueplayer.py
        # see: https://stackoverflow.com/questions/14262315/list-nearby-discoverable-bluetooth-devices-including-already-paired-in-python
        result = []
        for path in objects.keys():
            interfaces = objects[path]
            for interface in interfaces.keys():
                if interface == interface_name:
                    result.append(path)
        return result

    def run(self):
        "The main loop for the ROS node"
        rospy.Timer(rospy.Duration(10), self.pub_paired_devices)
        rospy.loginfo('Starting bluetooth_pair')
        while not rospy.is_shutdown():
            self.wait()

    def wait(self):
        # Run GTK events and sleep as the same rate as rospy.spin(), do not block
        # on the GTK iterate or rospy callbacks won't execute.
        gtk.main_iteration(block=False)
        rospy.rostime.wallsleep(0.5)

    def pub_paired_devices(self, event):
        "Publishes the currently-paired devices to the /paired_devices ROS topic"
        objects = self.manager.GetManagedObjects()
        devices = self.filter_by_interface(objects, DEVICE_IFACE)
        paired = BluetoothDevices()
        for device in devices:
            obj = self.proxyobj(self.bus, device, 'org.freedesktop.DBus.Properties')
            props = obj.GetAll(DEVICE_IFACE, dbus_interface='org.freedesktop.DBus.Properties')

            if 'Address' in props:
                mac_address = str(props['Address'])
            else:
                mac_address = None

            if 'Name' in props:
                device_name = str(props['Name'])
            else:
                device_name = 'Unnamed Device'

            if 'Paired' in props:
                is_paired = bool(props['Paired'])
            else:
                is_paired = False

            if is_paired:
                paired.devices.append(BluetoothDevice(mac_address=mac_address, device_name=device_name))
                #rospy.loginfo('Paired Device "{0}" [{1}]'.format(device_name, mac_address))

        self.list_paired_pub.publish(paired)

    # ROS Service calls
    def search_devices(self, req):
        "Create a background thread to discover nearby objects"
        if not self.discovery_in_progress:
            self.discovery_in_progress = True
            self.found_devices = dict()  # New search requested, clear old results.
            self.search_filter = req.device_name
            worker = threading.Thread(target=self.__discover_in_background)
            worker.start()
        else:
            rospy.logwarn("Bluetooth discovery scan already in progress; please wait for the first one to finish before starting a new one")
        return SearchDevicesResponse()

    def __discover_in_background(self):
        "The background worker thread invoked by search_devices(self, req)"
        rospy.loginfo('Starting Bluetooth discovery scan w/ filter "{0}"'.format(self.search_filter))
        devices = bluetooth.discover_devices(lookup_names=True)
        n_devices=0
        for device in devices:
            mac_address = device[0]
            device_name = device[1]

            if len(self.search_filter) == 0 or self.search_filter.lower() in device_name.lower():
                rospy.loginfo('Discovered device "{0}" [{1}] matching requested filter'.format(device_name, mac_address))
                bt_dev = BluetoothDevice(mac_address=mac_address, device_name=device_name)
                self.found_devices[mac_address] = bt_dev
                self.list_devices_pub.publish([v for (k, v) in self.found_devices.items()])
                n_devices = n_devices+1

        rospy.loginfo("Bluetooth discovery scan complete. Found {0} devices matching filter".format(n_devices))
        self.discovery_in_progress = False

    def pair_device(self, req):
        # Remove an already paired device, as the pair key may be
        # no longer valid, don't worry about any errors, just pass.
        try:
            device = self.adapter.FindDevice(req.device_mac)
            self.adapter.RemoveDevice(device)
            return PairDeviceResponse(True)
        except dbus.exceptions.DBusException:
            pass

        self.paired = False
        self.pair_error = False
        rospy.loginfo('Pairing with %s', req.device_mac)
        self.adapter.CreatePairedDevice(req.device_mac,
                                        self.dbus_path,
                                        'NoInputNoOutput',
                                        reply_handler=self.create_device_reply,
                                        error_handler=self.create_device_error)

        while not self.paired and not self.pair_error:
            self.wait()  # Keep GTK lib spinning

        if self.paired:
            rospy.loginfo('Trusting %s', req.device_mac)
            path = self.adapter.FindDevice(req.device_mac)
            device = dbus.Interface(self.bus.get_object('org.bluez', path),
                                    ADAPTER_IFACE)
            device.SetProperty('Trusted', dbus.Boolean(1))
            return PairDeviceResponse(True)
        elif self.pair_error:
            return PairDeviceResponse(False)

        # Fallthrough error
        return PairDeviceResponse(False)

    def remove_device(self, req):
        try:
            device = self.adapter.FindDevice(req.device_mac)
            self.adapter.RemoveDevice(device)
            return PairDeviceResponse(True)
        except dbus.exceptions.DBusException as e:
            rospy.logerr('Failed to remove device: %s', e)
            return PairDeviceResponse(False)

    # dbus callbacks
    def create_device_reply(self, device):
        rospy.loginfo('Paired with new device %s', device)
        self.paired = True

    def create_device_error(self, error):
        if 'Already Exists' in error.get_dbus_message():
            rospy.loginfo('Device already paired')
            self.paired = True
        else:
            rospy.logerr('Pairing failure: %s', error)
            self.pair_error = True
