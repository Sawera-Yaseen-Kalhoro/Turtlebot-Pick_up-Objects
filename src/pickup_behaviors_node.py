#!/usr/bin/env python

import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest
import operator
# from turtlebot_controller_node import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        # self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard = self.attach_blackboard_client(name="Blackboard")
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/check_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/check_object', Trigger)
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/check_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                self.blackboard.object_name = resp.message
                print("set to blackboard: ", resp.message)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/check_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/get_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/get_object', Trigger)
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [GetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/get_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/let_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/let_object', Trigger)
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [LetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# TODO: Create any other required behavior like those to move the robot to a point, 
#       add or check elements in the blackboard, ...
        
# Behavior to move the robot to a specified goal (from the Goal blackboard)
class Move(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Move, self).__init__(name)
        # Set blackboard client to Goal
        self.blackboard = self.attach_blackboard_client(name="Blackboard")
        self.blackboard.register_key(key="goal", access=py_trees.common.Access.READ)

        # Instatiate publisher
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # Setup to subscribe to the odometry (to get the robot's current pose)
        self.odom_sub = rospy.Subscriber("/odom",Odometry, self.get_odom)

        # Initialize current pose
        self.current_pose = (0,0)

    def setup(self):
        self.logger.debug("  %s [Move::setup()]" % self.name)
        try:
            self.logger.debug(
                "  %s [Move::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [Move::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Move::initialise()]" % self.name)
        self.goal = self.blackboard.goal # returning a tuple
        # publish to the goal topic
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = self.goal[0]
        goal_msg.pose.position.y = self.goal[1]
        self.goal_pub.publish(goal_msg)


    def update(self):
        try:
            print("Current pose = ",self.current_pose)
            print("Goal = ", self.goal)
            # check if the robot has reached the goal (with a 0.15 tolerance just like in the controller)
            distance_to_goal = np.linalg.norm(np.subtract(self.current_pose,self.goal))
            if distance_to_goal < 0.15:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        except:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Move::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    def get_odom(self,odom_msg):
        # odometry subscriber callback method
        # take only the x and y (we don't need the yaw here)
        self.current_pose = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)


# Behavior to set next goal point
class SetNextPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SetNextPoint, self).__init__(name)
        # Set blackboard client to Goal
        self.blackboard = self.attach_blackboard_client(name="Blackboard")
        self.blackboard.register_key(key="goal", access=py_trees.common.Access.WRITE)

        # Store a list of points to explore
        self.points_list = [(1.25,0.5),(1.25,-1.25),(0.0,-1.25),(-0.5,1.25),(-1.25,0.5)]
        self.next_point_index = 0

    def setup(self):
        self.logger.debug("  %s [SetNextPoint::setup()]" % self.name)
        try:
            self.logger.debug(
                "  %s [SetNextPoint::setup() Server connected!]" % self.name)
        except:
            self.logger.debug("  %s [SetNextPoint::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SetNextPoint::initialise()]" % self.name)

    def update(self):
        try:
            self.blackboard.goal = self.points_list[self.next_point_index]
            print("Set next goal point = ", self.blackboard.goal)
            self.next_point_index += 1
            return py_trees.common.Status.SUCCESS
        except:
            self.logger.debug(
                "  {}: Fail to update.".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [SetNextPoint::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior to increase the counter for objects done
class DoneCounter(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(DoneCounter, self).__init__(name)
        # Set blackboard client to Goal
        self.blackboard = self.attach_blackboard_client(name="Blackboard")
        self.blackboard.register_key(key="objects_done", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="objects_done", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [DoneCounter::setup()]" % self.name)
        try:
            # initialize counter in blackboard
            self.blackboard.objects_done = 0
            self.logger.debug(
                "  %s [DoneCounter::setup() Server connected!]" % self.name)
        except:
            self.logger.debug("  %s [DoneCounter::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [DoneCounter::initialise()]" % self.name)

    def update(self):
        try:
            self.objects_count = self.blackboard.objects_done
            self.objects_count += 1
            self.blackboard.objects_done = self.objects_count
            return py_trees.common.Status.SUCCESS
        except:
            self.logger.debug(
                "  {}: Fail to update.".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [DoneCounter::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

 # Function to create the behavior tree
def create_tree():
    # Create Behaviors
    check_object = CheckObject("Check object")
    get_object = GetObject("Get object")
    let_object = LetObject("Let object")
    move_to_next = Move("Move to next point")
    move_to_dropoff = Move("Move to drop-off")

    # Create behaviors to check and set blackboards
    check_done = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Both objects done?",
        check=py_trees.common.ComparisonExpression(
            variable="objects_done",
            value=2,
            operator=operator.eq
        )
    )

    not_both_objects_done = py_trees.decorators.Inverter(
        name="Inverter",
        child=check_done
    )

    set_next_point = SetNextPoint("Set next point")

    is_beer = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Object is beer?",
        check=py_trees.common.ComparisonExpression(
            variable="object_name",
            value="beer",
            operator=operator.eq
        )
    )

    set_beer = py_trees.behaviours.SetBlackboardVariable(
        name="Set drop-off point for beer",
        variable_name="goal",
        variable_value=(-1.5,-1.5),
        overwrite=True
    )

    set_coke = py_trees.behaviours.SetBlackboardVariable(
        name="Set drop-off point for coke",
        variable_name="goal",
        variable_value=(1.5,1.5),
        overwrite=True
    )

    # Behavior to increase the counter of the objects done
    done_counter = DoneCounter("Objects done +1")

    # Create composites from bottom up
    # Sequence to check and set for beer
    set_if_beer = py_trees.composites.Sequence(
        name="Sequence",
        memory=True,
        children=[is_beer, set_beer]
    )

    # Selector/Fallback to set drop-off point based on object (beer or coke)
    set_drop_off = py_trees.composites.Selector(
        name="Selector",
        memory=True,
        children=[set_if_beer,set_coke]
    )

    # Sequence of all actions (root)
    root = py_trees.composites.Sequence(
        name="Pickup Behavior",
        memory=True,
        children=[not_both_objects_done, set_next_point,move_to_next,check_object,get_object,set_drop_off,move_to_dropoff,let_object,done_counter]
    )

    return root
 
# Function to run the behavior tree
def run(it=5000):
    root = create_tree()

    try:
        print("Call setup for all tree children")
        root.setup_with_descendants() 
        print("Setup done!\n\n")
        py_trees.display.ascii_tree(root)
        
        for _ in range(it):
            root.tick_once()
            time.sleep(1)
    except KeyboardInterrupt:
        exit()
        # pass


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    # Create the behavior tree
    root = create_tree()
    
    # Display the behavior tree
    # py_trees.display.render_dot_tree(root, target_directory="catkin_ws/src/pick_up_objects_task/img") # generates a figure for the 'root' tree.

    # Run the behavior tree
    run()