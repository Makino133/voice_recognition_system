import sys
import os
import json
import vosk
import sounddevice as sd
import queue
import threading
import rclpy
import requests
import time
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from vosk import Model, KaldiRecognizer
from ament_index_python.packages import get_package_share_directory

# コマンドリスト
COMMAND_LIST = [
    "browser - open the browser",
    "terminal - create new pane",
    "list files - show the file list",
    "shutdown - shutdown the system",
    "bed - move to the bed",
    "table - move to the table"
]

#MODEL_PATH = ("/home/orin/ros2_ws/src/vint_ros/vosk-model")
MODEL_PATH = ("/home/orin/ros2_ws/src/vint_ros/vosk-model-lm/vosk-model-en-us-0.22")
model = Model(MODEL_PATH)

#API key and URL
API_KEY = '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'
API_URL = 'https://openrouter.ai/api/v1/chat/completions'

# Define the headers for the API request
headers = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
        }

q = queue.Queue()
def audio_callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

# ROS 2 ノード定義
class BrowserNode(Node):
    def __init__(self):
        super().__init__('browser_node')
        self.get_logger().info("Opening browser...")
        os.system("xdg-open https://www.google.com")

class TerminalNode(Node):
    def __init__(self):
        super().__init__('terminal_node')
        self.get_logger().info("Opening new terminal pane...")
        os.system("tmux split-window -h")

class ListFilesNode(Node):
    def __init__(self):
        super().__init__('list_files_node')
        self.get_logger().info("Listing files in terminal pane...")
        os.system("tmux split-window -h")
        os.system("tmux send-keys -t 1 'ls' Enter")

class ShutdownNode(Node):
    def __init__(self):
        super().__init__('shutdown_node')
        self.get_logger().info("Shutting down system via terminal...")
        os.system("tmux send-keys -t 1 'shutdown now' Enter")

class BedNode(Node):
    def __init__(self):
        goal = "bed"
        super().__init__('bed_node')
        print("Started BedNode") 

        self.cli = self.create_client(SetParameters, '/param_node/set_parameters')
    
        """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /param_node/set_parameters service...')
        """

        param = MsgParameter()
        param.name = 'GOAL'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=goal)
        req = SetParameters.Request()
        req.parameters = [param]
        self.future = self.cli.call_async(req)

        self.get_logger().info(f"Declared Parameter 'GOAL' with value: {goal}")

class TableNode(Node):
    def __init__(self):
        goal = "table"
        super().__init__('table_node')
        print("Started TableNode") 
            
        self.cli = self.create_client(SetParameters, '/param_node/set_parameters')
        
        """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /param_node/set_parameters service...')
        """

        param = MsgParameter()
        param.name = 'GOAL'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=goal)
        req = SetParameters.Request()
        req.parameters = [param]
        self.future = self.cli.call_async(req)

        self.get_logger().info(f"Declared Parameter 'GOAL' with value: {goal}")


# ROSノード起動関数
def launch_ros_node(node_class, spin=False):
    def target():
        node = node_class()
        if spin:
            rclpy.spin(node)
        else:
            rclpy.spin_once(node, timeout_sec=1.0)
    threading.Thread(target=target, daemon=True).start()

# 音声認識処理
class VoiceRecognition:
    def __init__(self):
        self.wake_word = "assistant"
        self.listening_for_command = False
        self.stop = False
        self.flag = 0
        print("===================================================")

    def run(self):
        #device_info = sd.query_devices(None, "input")
        #print(f"Default input device: {device_info['name']}")
        #print(f"Default sample rate: {device_info['default_samplerate']} Hz")
        #samplerate = int(sd.query_devices(None, "input")["default_samplerate"])
        samplerate = 16000
        with sd.RawInputStream(samplerate=samplerate, blocksize=16000, dtype="int16",
                               channels=1, callback=audio_callback, device=None):
            rec = vosk.KaldiRecognizer(model, samplerate)
            print("Listening for wake word...")

            while True:
                if self.stop == True:
                    continue
                start = time.time()
                data = q.get()
                end = time.time()
                time_diff = end - start
                #print("time1:",time_diff)

                audio_np = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                volume = np.sqrt(np.mean(audio_np**2))  # RMSボリューム

            
                # 閾値以下ならvoskに渡さない
                if volume < 0.01:
                    if self.listening_for_command  == True:
                        print("Listening...")
                    else:
                        print("Listening for wake word...")
                    continue
                

                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    text = result.get("text", "").strip().lower()
                    #print("time2:",time_diff)
                    #text = "assistant"
                    #text = input("test sentence: ")
                    print(f"audio recording: {text}")
                    if text:
                        print(f"(Detected: {text})")
                        if not self.listening_for_command and self.flag != 1:
                            if self.wake_word in text:
                                print("Assistant: Hello. What can I help you?")
                                self.listening_for_command = True
                        else:
                            self.stop = True
                            output = self.LLM(text)
                            self.process_command(output)
                            self.listening_for_command = False
                            if self.flag == 1:
                                print("Listening...")

    def LLM(self, text):
        base_prompt = """
        You are an assistant AI with a power wheelchair.
        Please adhere to the following conditions and return output.

        - From the last sentence, think and decide where I want to go.
        - Select the destination from the "word list".
        - Select from the options as much as possible. 
        - If the destination is determined, the output should always be this sentence and nothing else.
        "Move to the "(you selected word)""
        - If the destination cannot be determined, the word "again" is output. 
        - If the assistant reads that there is nothing to do, the word "end" is output.

        word lists: "bed", "table"

        My request:

        """

        text = input("test sentence: ")
        print(text)
        #text = "I am hungry"
        prompt = base_prompt + text
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        response = requests.post(API_URL, json=data, headers=headers)
        result = response.json()
        #print(result)
        output = result['choices'][0]['message']['content']
        print("Assistant:", output)
        return output

    def process_command(self, output):
        if "browser" in output:
            launch_ros_node(BrowserNode)
            print("Started BrowserNode")
            self.flag = 0
        elif "terminal" in output:
            launch_ros_node(TerminalNode)
            print("Started TerminalNode")
            self.flag = 0
        elif "list files" in output:
            launch_ros_node(ListFilesNode)
            print("Started ListFilesNode")
            self.flag = 0
        elif "shutdown" in output:
            launch_ros_node(ShutdownNode)
            print("Started ShutdownNode")
            self.flag = 0
        elif "bed" in output:
            launch_ros_node(BedNode)
            self.flag = 0
        elif "table" in output:
            launch_ros_node(TableNode)
            self.flag = 0
        elif "again" in output:
            print("Please say again")
            self.flag = 1
        elif "end" in output:
            self.flag = 0
        else:
            self.flag = 2
        self.stop = False

rclpy.init()
recognizer = VoiceRecognition()
recognizer.run()

if __name__ == "__main__":
    main()
