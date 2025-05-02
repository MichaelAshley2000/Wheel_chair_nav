import os
import json
import pyaudio
import wave
import base64
from dotenv import load_dotenv
from openai import OpenAI
from pydantic_models import ResponseFormat
import time
from playsound import playsound

load_dotenv()
client = OpenAI(api_key=os.getenv("API_KEY"))
response_file = "speech_to_cmd\\response.wav"


def record_audio(filename:str="command.wav", duration:int=5, rate:int=16000, channels:int=1):
    """This function records audio from the microphone and saves it to a WAV file."""
    chunk = 1024
    format = pyaudio.paInt16
    p = pyaudio.PyAudio()

    stream = p.open(format=format, channels=channels, rate=rate, input=True, frames_per_buffer=chunk)
    
    print("Recording...")
    frames = []
    for _ in range(0, int(rate / chunk * duration)):
        data = stream.read(chunk)
        frames.append(data)
    
    print("Finished recording")
    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(format))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()

def encode_audio_base64(filename:str="command.wav"):
    """This function encodes the audio file in base64 format."""
    with open(filename, "rb") as audio_file:
        wav_data = audio_file.read()
        encoded_string = base64.b64encode(wav_data).decode('utf-8')
    return encoded_string


# prompt = (
#         f"Imagine you are a smart intelligent system of a wheel chair. You only output navigational commands.\n"
#         "It is important that you identify the target location and action."
#         f"Parse this into a list of actions and targets in the following strict format:\n"
#         "[['action1', 'target1'], ['action2', 'target2'], ...]\n\n"
#         "For example, if the user says 'Go to the end table', the action and target should be:\n"
#         "[['move_to', 'end table']].\n\n"
#         "Another example: if the user says 'Go to the living room and find my mobile phone', the output should be:\n"
#         "[['move_to', 'living room'], ['find', 'mobile phone']].\n\n"
#         "Make sure to:\n"
#         "- Use clear actions like 'move_to', 'look_for_person', 'find', and 'follow'.\n"
#         "- Only include the specific action and target, with no extra text or information.\n"
#         "- The format must always be [['action', 'target']].\n"
#     )

def generate_audio_response(transcription):
    completion = client.chat.completions.create(
    model="gpt-4o-audio-preview",
    modalities=["text", "audio"],
    audio={"voice": "alloy", "format": "wav"},
    messages=[
        {
            "role": "system",
            "content": "Assume that you are a Smart wheel chair that can navigate to the places user wants to, Respond to the user's command in English."
        },
        {
            "role": "user",
            "content": transcription
        }
        ]
    )
    wav_bytes = base64.b64decode(completion.choices[0].message.audio.data)
    with open(response_file, "wb") as f:
        f.write(wav_bytes)
            
def transcribe_audio_realtime(encoded_string):
    try:
        completion = client.chat.completions.create(
            model="gpt-4o-audio-preview", 
            modalities=["text", "audio"],
            audio={"voice": "alloy", "format": "wav"},
            messages=[
                # {
                #     "role": "system",
                #     "content": prompt
                # },
                {
                    "role": "user",
                    "content": [
                        { 
                            "type": "text",
                            "text": "What is my command? If it is in Sinhala language, translate it to English."
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
        
        generate_audio_response(transcription)
            
        return transcription

    except Exception as e:
        print(f"An error occurred: {e}")
        return None

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
    return generated_text.model_dump()



def run_pipeline():
    filename = "command.wav"
    record_audio(filename=filename)
    encoded_audio = encode_audio_base64(filename=filename)
    start_time = time.time()
    transcription = transcribe_audio_realtime(encoded_audio)
    print(f"Time taken: {time.time() - start_time:.2f} seconds")

    if transcription:
        cmd_time = time.time()
        output_command = parse_command(transcription)
        print(f"Command generation time: {time.time() - cmd_time:.2f} seconds")
        playsound(response_file)
        with open("speech_to_cmd/output_command.json", "w") as f:
            json.dump(output_command, f, indent=2)
    
        print(f"output_command: {output_command}")
    else:
        print("No transcription available.")

if __name__ == "__main__":
    run_pipeline()


## General run time for the pipeline --- 6.4 seconds