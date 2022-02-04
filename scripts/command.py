#!/usr/bin/env python3
from cortex import Cortex
from emotiv_mc.msg import MentalCommand
import rospy

user = {
    "license" : "",
    "client_id" : "XXXX",
    "client_secret" : "XXXX",
    "debit" : 100
}

# name of profile
profile_name = 'pr2022'

class Command():

    def __init__(self):
        self.c = Cortex(user, debug_mode=True)
        self.c.bind(new_com_data=self.on_new_data)

    def do_prepare_steps(self):
        self.c.do_prepare_steps()

    def subscribe_data(self, streams):
        # streams : list, required list of streams. For example, ['com']
        self.c.sub_request(streams)

    def load_profile(self, profile_name):
        profiles = self.c.query_profile()

        if profile_name not in profiles:
            status = 'create'
            self.c.setup_profile(profile_name, status)

        status = 'load'
        self.c.setup_profile(profile_name, status)

    def unload_profile(self, profile_name):
        profiles = self.c.query_profile()

        if profile_name in profiles:
            status = 'unload'
            self.c.setup_profile(profile_name, status)
        else:
            print("The profile " + profile_name + " does not exist.")

    def on_new_data(self, *args, **kwargs):
        data = kwargs.get('data')
        # print(data["action"])
        msg = MentalCommand()
        msg.command = data["action"]
        msg.strength = data["power"]

        try:
            publish(msg)
        except rospy.ROSInterruptException: pass

#######################################################################

def publish(data):
        pub = rospy.Publisher('/mental_command', MentalCommand, queue_size=1)

        if not rospy.is_shutdown():
            pub.publish(data)

if __name__ == '__main__':

    rospy.init_node('emotiv_server', anonymous=True)

    # Init Command
    t=Command()

    # Do prepare steps
    t.do_prepare_steps()

    # load existed profile or create a new profile
    t.load_profile(profile_name)

    # subscribe sys stream to receive Training Event
    t.subscribe_data(['com'])

    # unload profile
    t.unload_profile(profile_name)

# -----------------------------------------------------------