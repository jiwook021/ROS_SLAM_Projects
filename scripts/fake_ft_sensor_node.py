#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped, Wrench
import numpy as np
import pandas as pd

class FakeSensor(object):
    #For making  Fake sensor data we need
    # 1. Publisher
    # 2. Wrench Stamped
    # 3. Fake Sensor Data
    # 4. Callback Function

    def __init__(self):
        self.sensor_data_count = 0
    
    def FakeFtSensorData(self, sensor_type = 0, a=2, b=0.05, n=1000):
        
        if(sensor_type == 0):
            dn = np.linspace(0, a*np.pi,n)
            fake_signal = b*np.sin(dn)
            self.fake_signal = fake_signal
        #Fake force data x,y,z
        """
        self.fake_result[0] = 
        self.fake_result[1] = 
        self.fake_result[2] = 
        """
        #Fake torque data x,y,z
        """
        self.fake_result[3] = 
        self.fake_result[4] = 
        self.fake_result[5] = fake_signal[self.sensor_data_count]
        """
    
    def GetFakeSensor(self,index):
        return self.fake_signal[index]


class FakeSensorClient(object):

    def __init__(self):
        '''
        Initializes the receiver class.
        port: The serial port to listen to.
        baudrate: Baud rate for the serial communication
        '''
        rospy.init_node('FakeSensor')  ## it tells rospy the name of your node

        #self._Publisher = rospy.Publisher('serial', String, queue_size=1)
        #self._Pub_ft_raw = rospy.Publisher('ft_raw', WrenchStamped, queue_size=1)
        self._Pub_ft_fake = rospy.Publisher('/ft_fake', WrenchStamped, queue_size=4)
        
        self.req_state = 0
        self.mode = 0
        self.callback_time_duration = 0.0005 # sec
        #self.result = [0.0]*6
        self.fake_result = [0.0]*6
        #self._SerialDataGateway = SerialDataGateway(port, baudrate,  self._HandleReceivedLine)
        #self._cmdReqeust = rospy.Service('ft_sensor/req_state', SetInt, self.callback_request_state)

        self.last_time = rospy.Time.now()
        self.cur_time = rospy.Time.now()
        self.max_elapsed = - 9999.0

        self._fakeSensor = FakeSensor()
        self._fakeSensor.FakeFtSensorData()

        # rospy timer Introduced in ROS 1.5, rospy provides a rospy.Timer convenience class which periodically calls a callback.
        # rospy.Timer(period, callback, oneshot=False)
        rospy.Timer(rospy.Duration(self.callback_time_duration), self._pub_ft_fake_data_callback)
        

    def _pub_ft_fake_data_callback(self, timer):
        data = WrenchStamped()
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = "ft_sensor_fake"
        data.wrench = Wrench()
        data.wrench.force.x = 0.0 #self.fake_result[0]
        data.wrench.force.y = 0.0
        data.wrench.force.z = 0.0
        
        data.wrench.torque.x = 0.0
        data.wrench.torque.y = self._fakeSensor.GetFakeSensor(self._fakeSensor.sensor_data_count)
        data.wrench.torque.z = 0.0

        self._Pub_ft_fake.publish(data)
        
        self.cur_time = rospy.Time.now()

        a = (self.cur_time - self.last_time).to_nsec()
        b = (self.callback_time_duration)*10.0**9

        self.max_elapsed = max(a, self.max_elapsed)
        print(a, type(a))
        print(b, type(b))

        if(a >= b):
            self._fakeSensor.sensor_data_count += 1      
            self.last_time = self.cur_time

        if(self._fakeSensor.sensor_data_count >= 1000):
            self._fakeSensor.sensor_data_count = 0

        print ('Timer Duration called at (ms) ' + str(a / 10.0**6) + 'max (ms) : ' + str(self.max_elapsed / 10.0**6))

if __name__ == '__main__':
    fake_sensor_client = FakeSensorClient()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        #ft_sensor.Stop()