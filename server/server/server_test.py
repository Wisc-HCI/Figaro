import eventlet
eventlet.monkey_patch()

from flask import Flask, request, jsonify, make_response, redirect
import json
import sys
import threading
from flask_cors import CORS
from flask_socketio import SocketIO, emit, Namespace

class Server (Namespace):
    def __init__(self,controller):
        super(Server,self).__init__()
        self.controller = controller

        self.layout='blank'
        self.undefinedIntent_query_dict = {'whole_text': 'robot say how can I help you today', 'text_class':['none', 'none', 'query', 'query', 'query', 'query', 'query', 'query'], 'agent': 'Robot', 'intent_categories': ['greeting', 'farewell', 'question', 'comment'], 'user_defined_intent_categories':['greeting', 'farewell'], 'user_defined_speech_examples':[['hello', 'sup', 'how are you?'], ['bye', 'see ya later']], 'robot_path':[[0.27812498807907104, 0.690625011920929, '', 'robot']], 'human_path':[[0.0390625, 0.5208333134651184, '', 'h1']], 'layout':''}
        #undefinedIntent_query_dict = {'whole_text': '', 'text_class': '', 'agent': '', 'intent_categories': '', 'user_defined_intent_categories':'', 'user_defined_speech_examples':''}
        self.conflict_query_dict = {'robot_action_list':'', 'event_env_list':'', 'behavior_option_list':'', 'robot_path':'', 'human_path':'', 'layout':''}
        self.query_index = 0
        self.query_list = []
        self.answer_list = []
        self.projectorX = 1
        self.projectorY = 1
        self.gridMultiplier = 3

    #handles saving and loading data
    def on_save_session(self,data):
        print("saving session")
        msg = String()
        msg.data = 'saved_data/save1'

    def on_load_session(self,data):
        print("loading session")
        msg = String()
        msg.data = 'saved_data/save1'

    #handles navigation messages received from tablet
    def on_tablet_trigger_nav(self,data):
        #check if layout needs updated
        if 'end_physicallayout' in data["message"]:
            self.updateProjectorImage(self.projectorX, self.projectorY, data["layout"], True)
        elif data["message"] == 'end_editspeechcats':
            print(data)

        if 'begin_deploy' not in data["message"]:
            #decide which page to navigate
            #if data["message"] != 'end_scene':
            temp = self.determine_next_page(data["message"])
            self.system_navigation(temp)

    #handles request for current layout
    def on_get_current_layout(self,data):
        self.updateProjectorImage(self.projectorX, self.projectorY, self.layout, True)

    #handles request for current list of scene summaries
    def on_tablet_request_scene(self,data):
        if(data["request"] == "all"):
            print("getting scene sumary list")
            self.controller.socketio.emit('all_scenes_summary', {'response':[]})
        else:
            print("getting one scene summary")
            #self.controller.publish_scene_info_request_pub(data["request"])

    def on_tablet_request_delete_scene(self,data):
        self.controller.publish_delete_scene_request(data["request"])

    def on_tablet_request_allintentcats(self,data):
        self.controller.publish_all_intent_cats_request()

    def on_deploy(self,data):
        self.controller.publish_message("tablet","begin_deploy")

    #handles request for current query information
    def on_get_query(self,data):
        print("getting query")
        if(data["request"] == "undefinedIntent"):
            self.undefinedIntent_query_dict['layout'] = self.layout
            self.controller.socketio.emit('undefinedIntent_query_stream', self.undefinedIntent_query_dict)
        elif(data["request"] == "conflict"):
            self.controller.socketio.emit('conflict_query_stream', self.conflict_query_dict)

    #handles return query from query screen
    def on_current_query_complete(self,data):
        if(data['type'] == "undefinedIntent"):
            #store result in answer_list[]
            self.answer_list.append([data['label'], data['speaker'], data['new_cat'], data['speech_examples'], data['edited_intent_categories'], data['edited_speech_examples']])

            #decide if call again or send ros message back
            if(self.query_index < len(self.query_list.queries) - 1):
                self.query_index = self.query_index + 1
                self.query_user(self.query_index)
            else:
                #return ros message
                msg = DataPassArray()
                for entry in self.answer_list:
                    temp = DataPass()
                    temp.destination = "ctrl"
                    temp.type = "DataPassUndefinedIntent"

                    uanswer = DataPassUndefinedIntent()

                    uanswer.label = entry[0]
                    uanswer.speaker = entry[1]

                    if(entry[2] == 'false'):
                        uanswer.new_cat= False
                    else:
                        uanswer.new_cat = True

                    sp_ex = StringArray()
                    for value in entry[3]:
                        sp_ex.array.append(value)
                    uanswer.speech_examples = sp_ex

                    ed_ic = StringArray()
                    for value in entry[4]:
                        ed_ic.array.append(value)
                    uanswer.edited_intent_categories = ed_ic

                    ed_se = []
                    for value in entry[5]:
                        intent_phrases = StringArray()
                        for phrase in value:
                            intent_phrases.array.append(phrase)
                        ed_se.append(intent_phrases)
                    uanswer.edited_speech_examples = ed_se

                    temp.undefined_intent_data = uanswer

                    msg.data_bundle.append(temp)

                self.controller.tablet_answer_publisher.publish(msg)
                self.system_navigation("sketch")
                self.controller.publish_message("tablet","end_query")
        elif(data["type"] == "conflict"):
            print("conflict resolution query complete")
            print(data["selection"])

            msg = DataPassArray()
            temp = DataPass()
            temp.destination = "ctrl"
            temp.type = "DataPassConflictResolution"

            answer = DataPassConflictResolution()
            answer.behavior_option = data["selection"]
            temp.conflict_resolution = answer
            msg.data_bundle.append(temp)

            self.controller.tablet_answer_publisher.publish(msg)
            self.system_navigation("sketch")
            self.controller.publish_message("tablet","end_query")

    #helper function: decides which page to switch to based on given nav message
    def determine_next_page(self,message):
        string = message.split("_")
        action = string[0]
        page = string[1]

        if action == "begin":
            #navigate to that page
            return page

        elif action == "end":
            #only sketch and physicallayout have end functions
            if page in ["sketch","physicallayout","deploy"]:#,"modify","insert"]:
                return "home"
            #elif page in ["rewind","fastforward","pause"]:
            #    return "scene"
            elif page in ["scene","viewscene","queryspeech","queryconflict","editspeechcats"]:
                return "sketch"
            else:
                #do nothing
                return "error"

        elif action == "cancel":
            #sub-modes from sketch
            #if page in ["rewind","fastforward","pause"]:

            #    return "scene"
            if page in ["scene","viewscene","editspeechcats"]:
                return "sketch"
            else:
                return "home"

        else:
            #do nothing
            return "error"

    #sends update of which page to navigate to
    def system_navigation(self,page):
        self.controller.socketio.emit('system_nav', {'response':page})
        print("emitted a system nav to {}".format(page))

    def send_save_to_ctrl(self):
        print("saving session")
        msg = String()
        msg.data = 'saved_data/save1'

        self.controller.save_publisher.publish(msg)

    def send_load_to_ctrl(self):
        print("loading session")
        msg = String()
        msg.data = 'saved_data/save1'

        self.controller.load_publisher.publish(msg)

    #sends the updated layout sketch grid to the ctrl node
    def send_layout_to_ctrl(self, objectData, regionData, x, y, env):
        msg = PhysicalLayout()
        msg.name = 'PhysicalLayout'
        msg.x = x
        msg.y = y
        msg.detailed_json = env

        for item in objectData:
            temp = ItemLocations()
            temp.name = list(item.keys())[0]
            for value in item[list(item.keys())[0]]:
                cell = Point()
                cell.x = float(value['x'])
                cell.y = float(value['y'])
                temp.data.append(cell)
            msg.object_data.append(temp)

        for item in regionData:
            temp = RegionLocations()
            temp.name = list(item[0].keys())[0]
            temp.r = float(item[1])
            temp.g = float(item[2])
            temp.b = float(item[3])
            temp.a = 1.0
            for value in item[0][list(item[0].keys())[0]]:
                cell = Point()
                cell.x = float(value['x'])
                cell.y = float(value['y'])
                temp.data.append(cell)
            msg.region_data.append(temp)
        print(msg)
        self.controller.layout_pub.publish(msg)
        #print(msg)

    def send_updated_cats_to_ctrl(self, edited_intent_cats, edited_speech_exs):
        msg = DataPassArray()

        temp = DataPass()
        temp.destination = "ctrl"
        temp.type = "DataPassAllIntentCategories"

        answer = DataPassAllIntentCategories()

        ed_ic = StringArray()
        for value in edited_intent_cats:
            ed_ic.array.append(value)
        answer.edited_intent_categories = ed_ic

        ed_se = []
        for value in edited_speech_exs:
            intent_phrases = StringArray()
            for phrase in value:
                intent_phrases.array.append(phrase)
            ed_se.append(intent_phrases)
        answer.edited_speech_examples = ed_se

        temp.all_intent_cats_data = answer

        msg.data_bundle.append(temp)
        print("passing all intent categories")
        self.controller.tablet_answer_publisher.publish(msg)

    def updateProjectorImage(self, x, y, env, flag):
        if(env != self.layout and env != 'na'):
            self.layout = env

        if((x != self.projectorX or y != self.projectorY) and (x != -1 and y != -1)): #this check ensures it will only update if there is a change
            self.projectorX = x
            self.projectorY = y

        self.controller.socketio.emit('layout_stream', {'layout': self.layout, 'projectorX': self.projectorX, 'projectorY': self.projectorY, 'update': flag, 'gridMultiplier':self.gridMultiplier})

    def updateGridMultiplier(self, x, y):
        print("updating grid multiplier to " + str(x/24))
        self.gridMultiplier = x/24

    def send_scene_summary(self,scene_summary):
        self.controller.socketio.emit('all_scenes_summary', {'response':[]})

    def send_individual_scene(self,r_path, h_path):
        self.controller.socketio.emit('individual_scene', {'response':True, 'robot_path':r_path, 'human_path':h_path, 'layout':self.layout})

    def send_all_intent_cats(self,cats, examples):
        print("sending all intent category data")
        intent_data = {'cats':cats, 'examples':examples}
        self.controller.socketio.emit('all_intent_cats',intent_data)

    #calls query screen for a given query
    def query_user(self,index):
        query = self.query_list.queries[index]

        # handle all undefined intent queries
        if query.type == "UndefinedIntentQuery":
            query_data = query.undefined_intent_query

            self.undefinedIntent_query_dict['whole_text'] = query_data.whole_text

            temp = []
            for item in query_data.text_class.array:
                temp.append(item)
            self.undefinedIntent_query_dict['text_class'] = temp

            self.undefinedIntent_query_dict['agent'] = query_data.agent

            temp = []
            for item in query_data.intent_categories.array:
                temp.append(item)
            self.undefinedIntent_query_dict['intent_categories'] = temp

            temp = []
            for item in query_data.user_defined_intent_categories.array:
                temp.append(item)
            self.undefinedIntent_query_dict['user_defined_intent_categories'] = temp

            temp = []
            for list in query_data.user_defined_speech_examples:
                temp2 = []
                for item in list.array:
                    temp2.append(item)
                temp.append(temp2)
            self.undefinedIntent_query_dict['user_defined_speech_examples'] = temp

            self.system_navigation("queryspeech")

        elif query.type == "ConflictQuery":
            print("conflict query detected")
            global conflict_query_dict
            query_data = query.conflict_query

            print("making robot actions")
            robot_actions = []
            for action in query_data.robot_action_list.array:
                robot_actions.append(action)

            print("making environment events")
            env_events = []
            for event in query_data.event_env_list.array:
                env_events.append(event)

            print("making options list")
            option_list = []
            for option in query_data.behavior_option_list.array:
                option_list.append(option)

            print("making robot/human paths")
            robot_path = []
            human_path = []
            for point in query_data.path.array:
                newPoint = [point.x, point.y, point.description, point.agent]
                if(point.agent == "robot"):
                    if(newPoint not in robot_path):
                        robot_path.append(newPoint)
                elif(point.agent == "h1"):
                    if(newPoint not in human_path):
                        human_path.append(newPoint)

            print("assembling dictionary")
            print("robot_action_list")
            self.conflict_query_dict['robot_action_list'] = robot_actions
            print("event_env_list")
            self.conflict_query_dict['event_env_list'] = env_events
            print("behavior_option_list")
            self.conflict_query_dict['behavior_option_list'] = option_list
            print("robot_path")
            self.conflict_query_dict['robot_path'] = robot_path
            print("human_path")
            self.conflict_query_dict['human_path'] = human_path

            print("layout")
            self.conflict_query_dict['layout'] = self.layout
            print("dictionary assembled")
            print(self.conflict_query_dict)
            print("sending nav message")
            self.system_navigation("queryconflict")


class Controller():
    def __init__(self):
        self.app = Flask(__name__)
        self.cors = CORS(self.app)
        self.server = Server(self)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.socketio.on_namespace(self.server)

    def receive_scene_summary_callback(self, msg):
        #CONVERT ROS MESSAGE TO PYTHON
        scene_summary = []
        print(msg)
        for summary in msg.array:
            scene_summary.append([summary.scene_number, summary.title , summary.duration_in_seconds , summary.summary])

        #send to tablet
        self.server.send_scene_summary(scene_summary)

    def receive_scene_info_callback(self, msg):
        print("received scene details")
        print(msg)

        robot_path = []
        human_path = []
        for point in msg.array:
            newPoint = [point.x, point.y, point.description, point.agent]
            if(point.agent == "robot"):
                if(newPoint not in robot_path):
                    robot_path.append(newPoint)
            elif(point.agent == "h1"):
                if(newPoint not in human_path):
                    human_path.append(newPoint)

        self.server.send_individual_scene(robot_path, human_path)

    def publish_delete_scene_request(self, scene_id):
        msg = String()
        msg.data = scene_id
        self.tablet_delete_scene_pub.publish(msg)

    '''
    OTHER SUBSCRIBERS
    '''
    def tablet_subscriber_callback(self, msg):
        '''
        PURPOSE: decide on what method to call based on the message received
        '''
        temp = self.server.determine_next_page(msg.data)
        self.server.system_navigation(temp)

        """if string == "begin_sketch":

            #self.begin_sketch_subscriber()
        elif string == "end_sketch":

            #self.end_sketch_subscriber()
        """

    def receive_load_session_callback(self, msg):

        if(msg.data != ''):
            print("received session data to load")
            self.server.updateProjectorImage(-1,-1, msg.data, True)
        else:
            print("no layout to load")

    def pixel_size_subsc_callback(self, msg):
        #first acknowledge received coords
        ack = String()
        ack.data = "tablet"
        self.pixel_size_ack.publish(ack)

        #now do something with it
        print("updating projected image size")
        print(msg)
        self.server.updateProjectorImage(msg.x, msg.y, 'na', False)

    def grid_size_subsc_callback(self, msg):
        print("received grid size")
        print(msg)
        self.server.updateGridMultiplier(msg.x, msg.y)

    def query_callback(self,msg):
        print("entering query_callback")
        #need to reset for each query, otherwise future queries send bogus data
        print(msg)
        self.server.answer_list = []

        #this will call the query screen for each query in the list
        self.server.query_list = msg

        self.server.query_index = 0
        self.server.query_user(self.server.query_index)


def main(args=None):
    node = Controller()

    if len(sys.argv) > 1:
        ip = sys.argv[1]
        print("host is {}".format(ip))
    else:
        ip = 'localhost'
        print("host is {}\nNOTE: to change the IP address, pass in the host as an argument".format(ip))

    #use host computer's IP address here
    #also make sure all ports used are allowed in firewall
    node.socketio.run(node.app, host=ip, port=7777)

if __name__=="__main__":
    main()
