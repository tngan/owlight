#!/usr/bin/env python
import rospy

# import self-defined module
from algo.lk_flow import OF
# farnback approach with control
from algo.fb_flow import OFC
# painless reconfigure
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

# global configuration

# 1. left threshold
# 2. right threshold
# 3. forward thrust
# 4. backward thrust
# 5. left angular thrust
# 6. right angular thrust
# 7. left thrust
# 8. right thrust

# a. lf config
# b. fb config

# eo gc

def inject_reconfigure(core):
    def callback(config, level):
        # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param}, {str_param}, {bool_param}, {size}""".format(**config))
        # update the global config here
        rospy.loginfo("Received reconf call: " + str(config))  
        core.update_reconfigure(config)
        return config
        #return config
    return callback

def main():
    rospy.init_node('owlight_node', anonymous=True, log_level=rospy.INFO)

    ddynrec = DDynamicReconfigure("owlight_ddy")
    ddynrec.add_variable("portion_threshold", "portion threshold", 0.5, 0.0, 1.0)
    ddynrec.add_variable("magnitude_threshold", "magnitude threshold", 2.0, 0.0, 7.5)
    ddynrec.add_variable("center_portion_flow", "center portion threshold", 0.3, 0.0, 1.0)
    ddynrec.add_variable("drift_degree", "drift degree (l/r control)", 0.7, 0.0, 2.0)
    ddynrec.add_variable("front_trust", "front trust", 0.3, 0.0, 1.0)
    ddynrec.add_variable("back_trust", "back trust", 0.3, 0.0, 1.0)
    ddynrec.add_variable("debug", "debug mode", True)
    ddynrec.add_variable("stop", "stop", True)
    ddynrec.add_variable("passive", "passive mode", False)

    core = OFC()
    ddynrec.start(inject_reconfigure(core))

    # srv = Server(, reconfigure_callback)
    # TODO: make OF becomes a generic function like OF(algoName)
    #OF()
    rate = rospy.Rate(10) # put it in launch file or dynamic configure later on
    rospy.loginfo('owlight node is being initialized, happy hacking')
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr('exception is caught in main')
