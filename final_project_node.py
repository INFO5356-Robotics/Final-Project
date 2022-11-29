from std_msgs.msg import String                 # Used to process ROS images
from geometry_msgs.msg import Twist             # Sends velocity commands to the robot
import playsound                                # Play .mp3 file
from gtts import gTTS                           # Text-to-speech
from irobot_create_msgs.msg import InterfaceButtons
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time

class FinalProject(Node):
    '''
    Node class that controls the robot, 
    stores relevant variables for the state machine, 
    social navigation, and text-to-speech during HRI.
    '''
    def __init__(self, user_position, dest_position, robot_expression):
        super().__init__('final_project_node')

        # Set initial parameters
        self.user_position = user_position
        self.dest_position = dest_position
        self.robot_expression = robot_expression
        
        # Create a publisher which can "talk" to robot and tell it to move
        self.movement_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            1)

        # Create a Twist message and add linear x and angular z values
        self.move_cmd = Twist()

        # Initialize navigation object
        self.navigator = TurtleBot4Navigator()
        
        # Set initial pose
        self.initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(self.initial_pose)

        # Initialize robot poses
        self.goal_pose = []

        # Define states for state machine
        self.state1 = 'approach_user'
        self.state2 = 'bump_or_bark'
        self.state3 = 'go_to_bowl'
        self.state4 = 'wait_for_reaction'
        self.state5 = 'end'

        # State Machine Variables
        self.curr_state = self.state1 # track current state
        self.next_state = self.state1 # track next state

        # Define which experiment we are running
        self.experiment = 1 # Bump into participant 
        #self.experiment = 2 # Bark at participant

    def update_state_machine(self):
        """Add Comments
        -------
        """
    
        if(self.curr_state == self.state1): # Approach User
                
            # Wait for Nav2
            self.navigator.waitUntilNav2Active()

            self.navigator.undock()
            
            # Set goal poses
            # goal_pose = []
            # User position
            self.add_goal(self.user_position['x'], self.user_position['y'],self.user_position['direction']) 
            # goal_pose.append(navigator.getPoseStamped([-3.058, -0.966], TurtleBot4Directions.NORTH))

            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)
            
            # Advance to next state
            self.next_state = self.state2
            
        elif(self.curr_state == self.state2): # Gesture 1(Bump into User) or Gesture 2(bark at user)
            # Wait for Nav2

            # Initialize navigation object
            self.navigator = TurtleBot4Navigator()

            if self.experiment == 1: # Gesture 1
                # Set goal poses
                self.goal_pose = []
                # Destination1 position 
                self.add_goal(self.dest_position['x1'], self.dest_position['y1'],self.user_position['direction'])

                # Follow Waypoints
                self.navigator.startFollowWaypoints(self.goal_pose)

            if self.experiment == 2: # Gesture 2
                self.robot_talker(robot_phrase='Bark')

            # Advance to next state
            self.next_state = self.state3

        elif(self.curr_state == self.state3): # Go To Bowl
            # Wait for Nav2
            self.navigator = TurtleBot4Navigator()

            # Set goal poses
            self.goal_pose = []
            # Destination1 position 
            self.add_goal(self.dest_position['x2'], self.dest_position['y2'],self.user_position['direction'])
            # goal_pose.append(navigator.getPoseStamped([-0.579, 0.004], TurtleBot4Directions.NORTH)) #THIS NEEDS TO BE CHANGED TO A NEW WAYPOINT AS CLOSE TO DOCK AS YOU CAN

            # Follow Waypoints
            self.navigator.startFollowWaypoints(self.goal_pose)

            # Finished navigating to docking station
            self.navigator.dock()

            # Advance to next state
            self.next_state = self.state4

        elif(self.curr_state == self.state4): # Wait at bowl for user 
            # Robot alerts user that it is done
            self.move_robot(x=0.0, z=1.5)
            
            # Advance to next state
            self.next_state = self.state5
        
        elif(self.curr_state == self.state5): # React to user 
            # Robot alerts user that it is done
            self.robot_talker(robot_phrase='Thank you for participating in the experiment')
            
            # End state machine
            self.next_state = None
        
        # Advance to next state
        self.curr_state = self.next_state

    def robot_talker(self, robot_phrase='Welcome to human robot interaction', output_filename='robot_talker.mp3'):
        """Uses text to speech software to enable to robot to 
            alert users when they are in the intimate and public zones                                                    
        ----------
        robot_phrase : robot phrase
            String of text phrase 
        output_filename : name of file to store audio file
            String of outputfile name
        Returns
        -------
        None
        """
        # Language in which you want to convert
        language = 'en'
        
        # Passing the text and language to the engine, 
        # here we have marked slow=False. Which tells 
        # the module that the converted audio should 
        # have a high speed
        myobj = gTTS(text=robot_phrase, lang=language, slow=False)
        
        # Saving the converted audio in a mp3 file named
        # welcome 
        myobj.save(output_filename)

        # Play audio file with playsound library
        playsound.playsound(output_filename, True)
    
    def move_robot(self, x=1.5, z=1.5, clockwise=True):
        """Move the robot using x and z velocities
        ----------
        x : float
            linear x velocity.
        z : float
            angualr z velocity.
        clockwise : bool
            True - rotate right, False - rotate left
        Returns
        -------
        None
        """

        self.move_cmd.linear.x = float(-x) # back or forward
        # if(clockwise):
        #     self.move_cmd.angular.z = float(-z)
        # else:
        #     self.move_cmd.angular.z = float(z)
        while z < 5:
            self.movement_pub.publish(self.move_cmd)
            z += 0.5
            if(clockwise):
                self.move_cmd.angular.z = float(-z)
            else:
                self.move_cmd.angular.z = float(z)

        self.move_cmd.linear.x = float(x)
        
    def add_goal(self, x, y, direction):
        '''`
        x : robot pose in x direction
        y : robot pose in y direction
        direction : orientation about the z axis in degress
        Returns None
        '''
        self.goal_pose.append(self.navigator.getPoseStamped([x,y], direction))

def main():
    rclpy.init()

    # Set goal poses
    user_position = {'x':-0.03, 'y':-2.99, 'direction':TurtleBot4Directions.NORTH}
    dest_position = {'x1':0.96, 'y1':-5.45, 'direction1':TurtleBot4Directions.NORTH,
                     'x2':-0.02, 'y2':-0.94, 'direction2':TurtleBot4Directions.NORTH}

    # Set expressive behavior for robot. Options include gesture, or both
    robot_expression = 'gesture' # 'gesture'

    # Intiialize tour robot
    tour_robot = FinalProject(user_position, dest_position, robot_expression)

    # Run state_machine
    while(1):
        tour_robot.update_state_machine()
        if(tour_robot.curr_state is None):
            break

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()