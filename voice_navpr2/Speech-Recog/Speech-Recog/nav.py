#!/usr/bin/env python3

import os
import sys
import random
import threading
import base64
import wave
import pyaudio
import rospy
import actionlib
from dotenv import load_dotenv
from openai import OpenAI

# ROS messages and services
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from pydantic import BaseModel, Field
import json

# PyDub for audio playback
from pydub import AudioSegment
from pydub.playback import play

load_dotenv()
openai_api_key = os.getenv("API_KEY")
if not openai_api_key:
    print("Error: API_KEY not found in .env file.")
    sys.exit(1)

client = OpenAI(api_key=openai_api_key)

CHUNK_SIZE = 1024
AUDIO_FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
RECORD_DURATION = 5
COMMAND_FILENAME = "command.wav"
RESPONSE_FILENAME = "response.wav"

# Region boundaries for mapping
regions = {
    "kitchen": {"x_min": 0.5,  "x_max": 3, "y_min": 1,    "y_max": 2,  "fallback": (1.25, 1.25)},
    "bed_room": {"x_min": -4,  "x_max": -1, "y_min": 1, "y_max": 3,    "fallback": (1.25, -1.25)},
    "toilet": {"x_min": -3, "x_max": -1,   "y_min": 6.5,    "y_max": 7.5,  "fallback": (-1.5, 1.25)},
    "living_room": {"x_min": -3, "x_max": -1,   "y_min": 3, "y_max": 4.5,    "fallback": (-1.5, -1.25)},
}

class Position(BaseModel):
    x: float = Field(
        ...,
        description="X coordinate of the position on the map."
    )
    y: float = Field(
        ...,
        description="Y coordinate of the position on the map."
    )

class ResponseFormat(BaseModel):
    transcript: str = Field(
        ...,
        description="The recognized or translated user input text."
    )
    location: str = Field(
        ...,
        description="The target location of the command. Location must be one of the following: 'kitchen', 'bed_room', 'toilet', 'living_room'."
    )
    command_type: str = Field(
        ...,
        description="The type of command, e.g., 'navigation'."
    )
    destination: Position = Field(
        ...,
        description="The target position (x, y) on the map. For kitchen its (4.0, 5.0). For bed_room its (3.0, 1.0)"
    )
    navigation_mode: str = Field(
        ...,
        description="Navigation mode, e.g., 'safe', 'fast', 'smooth'."
    )
    obstacle_avoidance: bool = Field(
        ...,
        description="Whether the system should perform obstacle avoidance. (True/False)"
    )
    priority: str = Field(
        ...,
        description="Priority level for this command, e.g., 'normal' or 'high'."
    )
  

def record_audio(filename=COMMAND_FILENAME, duration=RECORD_DURATION, rate=RATE, channels=CHANNELS):
    """
    Record audio from the microphone and save it to a WAV file.
    """
    p = pyaudio.PyAudio()
    stream = p.open(format=AUDIO_FORMAT,
                    channels=channels,
                    rate=rate,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE)

    print("Recording...")
    frames = []
    for _ in range(int(rate / CHUNK_SIZE * duration)):
        data = stream.read(CHUNK_SIZE)
        frames.append(data)
    print("Finished recording.")

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(AUDIO_FORMAT))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()


def encode_audio_base64(filename=COMMAND_FILENAME):
    """
    Encode the recorded audio (WAV) into base64 format for sending to OpenAI.
    """
    with open(filename, "rb") as audio_file:
        wav_data = audio_file.read()
    return base64.b64encode(wav_data).decode('utf-8')


def generate_audio_response(transcription, filename=RESPONSE_FILENAME):
    """
    Generate TTS audio from OpenAI and immediately play it using PyDub.
    """
    try:
        completion = client.chat.completions.create(
            model="gpt-4o-audio-preview",  # Example model name
            modalities=["text", "audio"],
            audio={"voice": "alloy", "format": "wav"},
            messages=[
                {
                    "role": "system",
                    "content": "Assume you are a smart wheelchair. Provide responses based on the user's navigation commands."
                },
                {
                    "role": "user",
                    "content": transcription
                }
            ]
        )
        # Decode base64 TTS and save to WAV
        wav_bytes = base64.b64decode(completion.choices[0].message.audio.data)
        with open(filename, "wb") as f:
            f.write(wav_bytes)

        # Play the TTS response
        sound = AudioSegment.from_wav(filename)
        play(sound)

    except Exception as e:
        print(f"Error generating audio response: {e}")


def transcribe_audio_realtime(encoded_string):
    """
    Transcribe the recorded audio via OpenAI API (example gpt-4o-audio-preview).
    """
    try:
        completion = client.chat.completions.create(
            model="gpt-4o-audio-preview",
            modalities=["text", "audio"],
            audio={"voice": "alloy", "format": "wav"},
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "Understand and Translate my sinhala navigation voice command to English. If given some task, try to understand the location where that task will be performed. Destination should be one of the following: 'kitchen', 'bed_room', 'toilet', 'living_room'."
                            # "text": "What is my command? If it is in Sinhala, translate it to English."
                        },
                        {
                            "type": "input_audio",
                            "input_audio": {
                                "data": encoded_string,
                                "format": "wav"
                            }
                        }
                    ]
                },
            ],
        )
        transcription = completion.choices[0].message.audio.transcript
        print(f"Transcription: {transcription}")
        # Generate TTS response
        generate_audio_response(transcription)
        return transcription

    except Exception as e:
        print(f"An error occurred during transcription: {e}")
        return None


def is_valid_point(x, y, map_data):
    """
    Check if a point (x, y) is valid (within bounds & free of obstacles).
    """
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y
    width = map_data.info.width
    height = map_data.info.height

    # Convert (x, y) to map coordinates
    map_x = int((x - origin_x) / resolution)
    map_y = int((y - origin_y) / resolution)

    # Check if the point is within the map bounds
    if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
        return False

    # Check if the point is free space
    index = map_y * width + map_x
    if map_data.data[index] == -1 or map_data.data[index] > 65:
        return False

    return True


def get_random_point(region, map_data):
    """
    Get a random valid point in the specified region or a fallback point if no valid point is found.
    """
    if region not in regions:
        rospy.logerr(f"Invalid region: {region}")
        return None

    region_data = regions[region]
    for _ in range(100):  # Try 100 times
        x = random.uniform(region_data["x_min"], region_data["x_max"])
        y = random.uniform(region_data["y_min"], region_data["y_max"])
        if is_valid_point(x, y, map_data):
            return x, y

    # Fall back to predefined if no valid point is found
    rospy.logwarn(f"No valid random point found in region {region}. Using fallback point.")
    return region_data["fallback"]

def move_to_goal(x, y):
    """
    Move the robot to the goal position (x, y).
    """
    # Initialize the move_base action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Wait for the action server to start
    client.wait_for_server()

    # Create a new goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Reference frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal position and orientation
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # No rotation (adjust if needed)

    # Send the goal
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check the result
    result = client.get_result()
    if result:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Failed to reach the goal.")
        
def move_to_goal(x, y, z=0.0, w=1.0):
    """
    Send the robot to a specified goal using move_base action.
    """
    mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    mb_client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    rospy.loginfo(f"Sending goal: x={x}, y={y}")
    mb_client.send_goal(goal)
    mb_client.wait_for_result()
    return mb_client.get_result()


# -------------------------------------------------------------------------
#               3. Region Markers for Visualization in RViz
# -------------------------------------------------------------------------

def create_region_marker(region_name, points, frame_id, marker_id):
    """
    Create a LINE_STRIP marker outlining a region's boundary.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "regions"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # line width
    marker.color.a = 0.6  # transparency
    marker.color.r = 1.0  # red color for the boundary

    for pt in points:
        p = Point()
        p.x = pt[0]
        p.y = pt[1]
        p.z = 0
        marker.points.append(p)

    # Close the loop by re-adding the first point
    first_pt = points[0]
    p = Point()
    p.x = first_pt[0]
    p.y = first_pt[1]
    p.z = 0
    marker.points.append(p)

    return marker


def create_region_label(region_name, x, y, frame_id, marker_id):
    """
    Create a TEXT_VIEW_FACING marker labeling a region.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "regions"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.scale.z = 0.5  # Text size
    marker.color.a = 1.0  # full opacity
    marker.color.b = 1.0  # label color: blue
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.text = region_name

    return marker


def publish_region_markers():
    """
    Continuously publish region markers (boundaries + labels) at 1 Hz.
    """
    regions = {
    # right wing → kitchen
        "kitchen": [
            (0,  2.6),
            (4.0,  3.0),
            (4.3, 0.3),
            (0.6, -0.4),
        ],

        # top leg → toilet
        "toilet": [
            (-2.8,  6),
            (-3.1,  8),
            (-1.1,  8.3),
            (-0.7,  6.2),
        ],

        # lower-left wing → bed room
        "bed_room": [
            (-4, 1.0),
            (-4, 3.0),
            (-1,  3),
            (-1,  1),
        ],

        # upper-left wing → living room
        "living_room": [
            (-4,  3),
            (-4,  5),
            (-1,  5),
            (-1,  3),
        ],
    }


    # Create a publisher for visualization markers
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz for publishing markers

    try:
        while not rospy.is_shutdown():
            marker_array = MarkerArray()
            marker_id = 0

            for region_name, pts in regions.items():
                # Boundary Marker
                region_marker = create_region_marker(region_name, pts, "map", marker_id)
                marker_array.markers.append(region_marker)
                marker_id += 1

                # Label Marker
                centroid_x = sum(pt[0] for pt in pts) / len(pts)
                centroid_y = sum(pt[1] for pt in pts) / len(pts)
                label_marker = create_region_label(region_name, centroid_x, centroid_y, "map", marker_id)
                marker_array.markers.append(label_marker)
                marker_id += 1

            # Publish the marker array
            marker_pub.publish(marker_array)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Marker publishing interrupted. Shutting down.")


def marker_thread():
    """
    Helper thread to continuously publish region markers.
    """
    publish_region_markers()


# -------------------------------------------------------------------------
#                  4. Voice Navigation and Execution
# -------------------------------------------------------------------------

def parse_command(transcription):
    prompt = (f"Imagine you are a smart intelligent system of a wheel chair. Provide a structured output based on the user's command.")
    messages = [
        {
         "role": "system",
         "content": prompt,
        },
        ] 

    response = client.beta.chat.completions.parse(
        model="gpt-4o-mini",  
        messages= messages + [{"role": "user", "content": transcription}],
        temperature=0.5,
        response_format=ResponseFormat
    )
    
    generated_text = response.choices[0].message.parsed
    output_command = generated_text.model_dump()
    return output_command['location']


def navigate_based_on_voice():
    """
    Main process: record audio, transcribe, parse region, navigate, generate TTS response.
    """
    # 1) Record voice
    record_audio()

    # 2) Encode audio to base64
    encoded_audio = encode_audio_base64()

    # 3) Send to OpenAI for transcription
    transcription = transcribe_audio_realtime(encoded_audio)
    if not transcription:
        rospy.logerr("No transcription result. Aborting navigation.")
        return

    rospy.loginfo(f"Transcription: {transcription}")

    # 4) Parse the command to identify the region
    region = parse_command(transcription)
    if not region:
        rospy.logerr("No valid region recognized in the transcription.")
        return

    # 5) Get the map
    rospy.wait_for_service('static_map')
    try:
        get_map = rospy.ServiceProxy('static_map', GetMap)
        map_data = get_map().map
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call static_map: {e}")
        return

    # 6) Get a random valid point in the region
    goal_point = get_random_point(region, map_data)
    if not goal_point:
        rospy.logerr(f"Could not find a valid point in region {region}")
        return

    x, y = goal_point
    rospy.loginfo(f"Navigating to region {region} at ({x}, {y})")

    # 7) Send the goal to move_base
    result = move_to_goal(x, y)
    if result:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.logwarn("Failed to reach the goal.")


# -------------------------------------------------------------------------
#                          5. Entry Point
# -------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        # 1) Initialize ROS node
        rospy.init_node('voice_navigation', anonymous=True)

        # 2) Start region markers in a separate thread
        threading.Thread(target=marker_thread, daemon=True).start()

        # 3) Perform voice-based navigation
        navigate_based_on_voice()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted. Exiting.")