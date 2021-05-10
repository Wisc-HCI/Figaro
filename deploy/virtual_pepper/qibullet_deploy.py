import pybullet as p
import time
import math
import pybullet_data
import threading
from qibullet import PepperVirtual
from qibullet import SimulationManager

import xml.etree.ElementTree as ET

class State:
    def __init__(self,n,n_next='na',tstep='na',x1='na',x2='na',y1='na',y2='na',t1='na',t2='na',lab='none'):
        self.id = n
        self.next_id = n_next
        self.time_step = tstep
        self.x_start = x1
        self.y_start = y1
        self.theta_start = t1 - math.pi/2
        self.x_end = x2
        self.y_end = y2
        self.theta_end = t2 - math.pi/2
        self.label = lab

    #TODO: this needs to be adjusted to a local velocity
    def get_velocity(self):
        global velocity_scale
        if self.label == 'none':
            xdiff = self.x_end - self.x_start#(x2-x1)
            ydiff = self.y_end - self.y_start#(y2-y1)
            vel = math.sqrt(xdiff**2 + (ydiff)**2) / self.time_step

            return vel
        else:
            return 1

    def check_transition_condition(self,x,y,t):
        print("check",t,self.theta_end,t-self.theta_end)
        if(abs(x-self.x_end)<0.1 and abs(y-self.y_end)<0.1):# and abs(t-self.theta_end)<0.1):
            print("reached goal, go to next state")
            return self.next_id
        else:
            return self.id


currentGesture = ""
velocity_scale = 1
def wave(robot):
    joint_positions = [[0.75, -0.75, -0.05, 0,0,0], [0.15, -0.75, -1.25, 0,0,0], [0.75, -0.75, -0.05, 0,0,0],[0,1.2,0, 0,0,0]]
    global currentGesture
    for config in joint_positions:
        if(currentGesture == "wave"):
            print("next wave point")
            robot.setAngles(["LShoulderRoll", "LShoulderPitch", "LElbowRoll","LFinger11","LFinger12","LFinger13"],config,0.3)
            time.sleep(0.9)
        else:
            return 0

    currentGesture = ""
    return 0

def point(robot):
    robot.setAngles(["LShoulderRoll", "LShoulderPitch", "LElbowRoll","LFinger11","LFinger12","LFinger13"],[0,0,0,0.85,0.85,0.85],0.2)
    time.sleep(3)
    currentGesture = ""

def neutral(robot):
    robot.setAngles(["LShoulderRoll", "LShoulderPitch", "LElbowRoll","LFinger11","LFinger12","LFinger13"],[0,1.2,0, 0,0,0],0.2)

def gesture_management(robot):
    while True:
        global currentGesture
        if(currentGesture == "wave"):
            print("waving")
            wave(robot)
            print("done waving")
        elif(currentGesture == "point"):
            print("pointing")
            point(robot)
            print("done pointing")
        else:
            neutral(robot)

def main():
    ###variables
    temp_velocity_scale = 1
    human_pos = {'x':0, 'y':0}
    grid_scale_factor = 40

    #first, parse the XML file and build the state machine
    tree = ET.parse('interaction_p.xml')
    root = tree.getroot()

    states = {}
    for state in root.iter('state'):
        n = state.get('id')
        if state.find("./behaviors//*[@cat='sys']//*[@val='ON']") != None:
            print("initial state")
            transition = root.find("./transition[@source_id='" + n + "']")
            n_next = transition.get('target_id')
            print(n, "->",n_next)

            env_state = transition.find("./env_state")
            x = float(env_state.find("*[@label='xpos']/val").get('val'))/grid_scale_factor
            y = float(env_state.find("*[@label='ypos']/val").get('val'))/grid_scale_factor
            t = float(env_state.find("*[@label='rotation']/val").get('val'))*math.pi/180
            print("angle",t)

            human_pos["x"] = float(env_state.find("*[@label='h_xpos']/val").get('val'))/grid_scale_factor
            human_pos["y"] = float(env_state.find("*[@label='h_ypos']/val").get('val'))/grid_scale_factor

            state_class = State('initial',n_next,x1=x,y1=y,t1=t,x2=x,y2=y,t2=t,lab='initial')
            states['initial'] = state_class

        elif state.find("./behaviors//*[@cat='sys']//*[@val='OFF']") != None:
            print("final state")
            #these come from previous state
            prev_state = root.find("./transition[@target_id='" + n + "']")
            x = float(prev_state.find("./env_state//*[@label='xpos']/val").get('val'))/grid_scale_factor
            y = float(prev_state.find("./env_state//*[@label='ypos']/val").get('val'))/grid_scale_factor
            t = float(prev_state.find("./env_state//*[@label='rotation']/val").get('val'))*math.pi/180
            print("end at",x,y,t)

            state_class = State(n,n_next,x1=x,y1=y,t1=t,x2=x,y2=y,t2=t,lab='final')
            states[n] = state_class

        else:
            transition = root.find("./transition[@source_id='" + n + "']")
            n_next = transition.get('target_id')
            print(n, "->",n_next)
            tstep = float(state.find("./behaviors//*[@cat='movement_time']/value").get('val'))

            #these come from previous state
            prev_state = root.find("./transition[@target_id='" + n + "']")
            x1 = float(prev_state.find("./env_state//*[@label='xpos']/val").get('val'))/grid_scale_factor
            y1 = float(prev_state.find("./env_state//*[@label='ypos']/val").get('val'))/grid_scale_factor
            t1 = float(prev_state.find("./env_state//*[@label='rotation']/val").get('val'))*math.pi/180

            #these come from current state, we get them from the transition because none are missing
            env_state = transition.find("./env_state")
            x2 = float(env_state.find("*[@label='xpos']/val").get('val'))/grid_scale_factor
            y2 = float(env_state.find("*[@label='ypos']/val").get('val'))/grid_scale_factor
            t2 = float(env_state.find("*[@label='rotation']/val").get('val'))*math.pi/180
            print(x1,y1,t1," to ",x2,y2,t2)

            state_class = State(n,n_next,tstep,x1,x2,y1,y2,t1,t2)
            states[n] = state_class

            n = state_class.get_velocity()
            if abs(n) > temp_velocity_scale:
                temp_velocity_scale = abs(n)

    #we need to scale the speed of pepper based on the max speed from the traces
    global velocity_scale
    velocity_scale = temp_velocity_scale
    print("velocity scale",velocity_scale)

    #then, start the simulation
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane100.urdf")

    roomId = p.loadURDF("white_room.urdf", [5,7.5,-0.05], p.getQuaternionFromEuler([math.pi/2, 0, -math.pi/2]), globalScaling=0.55,useFixedBase=True)

    humanoidId = p.loadURDF("humanoid/humanoid.urdf", [human_pos['x'],human_pos['y'],0.85], p.getQuaternionFromEuler([math.pi/2,0,math.pi/2]), globalScaling=0.24, useFixedBase=True)

    pepper = PepperVirtual()
    pepper.loadRobot(
      translation=[states['initial'].x_end, states['initial'].y_end, 0],
      quaternion=p.getQuaternionFromEuler([0,0,states['initial'].theta_end]),
      physicsClientId=physicsClient)

    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)

    input("Press Enter to continue...")

    p.setRealTimeSimulation(True)

    gesture_thread = threading.Thread(target=gesture_management,kwargs=dict(robot=pepper), daemon=True)
    gesture_thread.start()

    current_state = 'initial'
    flag = True
    final_state = ''
    while(flag):
        x = states[current_state].x_end
        y = states[current_state].y_end
        t = states[current_state].theta_end
        v = states[current_state].get_velocity()
        t1 = states[current_state].theta_start
        if(t1 == 'na'):
            t1 = t
        if((t - t1)>math.pi):
            t = -(2*math.pi - t)

        pepper.moveTo(x,y,t,frame=1,speed=v,_async=True)
        time.sleep(0.01)

        xcurr, ycurr, tcurr = pepper.getPosition()
        print("current (" + str(current_state) + "): " + str(xcurr) + " " + str(ycurr) + " " + str(tcurr))
        print("goal",x,y,t)
        print("theta",t1*180/math.pi,t*180/math.pi)
        current_state = states[current_state].check_transition_condition(xcurr,ycurr,tcurr)

        if(states[current_state].label == 'final'):
            print('final state reached, exiting state machine')
            flag=False

    #pepper.move(0,0,0)

    for i in range(100):
        print(i)
        time.sleep(1)

    p.disconnect()

if __name__ == "__main__":
    main()
