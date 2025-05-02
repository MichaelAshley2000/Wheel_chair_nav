import openai
import os
import pyaudio
import wave
import base64
from dotenv import load_dotenv
from openai import OpenAI
from langchain import LLMChain
from langchain.prompts import PromptTemplate
from langchain.llms import OpenAI as LangOpenAI
from langchain.callbacks import get_openai_callback
import json

# Load the .env file
load_dotenv()

# Access the OpenAI API key
openai_api_key = os.getenv("API_KEY")
openai.api_key = openai_api_key

client = OpenAI(api_key=openai_api_key)

# Record audio in real-time
def record_audio(filename="command.wav", duration=5, rate=16000, channels=1):
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

    # Save the recording to a WAV file
    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(format))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()

# Encode the WAV file in base64
def encode_audio_base64(filename="command.wav"):
    with open(filename, "rb") as audio_file:
        wav_data = audio_file.read()
        encoded_string = base64.b64encode(wav_data).decode('utf-8')
    return encoded_string

# Send the audio and text to OpenAI's Realtime Console for transcription
def transcribe_audio_realtime(encoded_string):
    try:
        # OpenAI Realtime Console request
        completion = client.chat.completions.create(
            model="gpt-4o-audio-preview",  # Replace with the correct real-time model identifier
            modalities=["text", "audio"],
            audio={"voice": "alloy", "format": "wav"},
            messages=[
                {
                    "role": "user",
                    "content": [
                        { 
                            "type": "text",
                            "text": "What is my command? If it is in Sinhala language, translate it to English"
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
            ]
        )
        # with open("completion_response.json", "w") as f:
        #     json.dump(completion.choices[0].message.to_dict(), f, indent=4)
        transcription = completion.choices[0].message.audio.transcript
        print(f"Transcription: {transcription}")
        return transcription

    except Exception as e:
        print(f"An error occurred: {e}")
        return None

# Parse the command using GPT-3.5 model with Langchain and trace cost and tokens
def parse_command(transcription):
    print(f"Transcription: {transcription}")

    prompt = (
        f"Imagine you are a smart intelligent system of a wheel chair. You only output navigational commands.\n"
        "It is important that you identify the target location and action."
        f"Parse this into a list of actions and targets in the following strict format:\n"
        "[['action1', 'target1'], ['action2', 'target2'], ...]\n\n"
        "For example, if the user says 'Go to the end table', the action and target should be:\n"
        "[['move_to', 'end table']].\n\n"
        "Another example: if the user says 'Go to the living room and find my mobile phone', the output should be:\n"
        "[['move_to', 'living room'], ['find', 'mobile phone']].\n\n"
        "Make sure to:\n"
        "- Use clear actions like 'move_to', 'look_for_person', 'find', and 'follow'.\n"
        "- Only include the specific action and target, with no extra text or information.\n"
        "- The format must always be [['action', 'target']].\n"
        f"The transcription of the user's command: '{transcription}'.\n" 
        "Now, parse the user's command:"
    )
    print(prompt)
    # Use OpenAI API to get a response
    messages = [
        {
         "role": "system",
         "content": prompt,
        },
        ] 

    response = client.chat.completions.create(
        model="gpt-3.5-turbo",  # Use your desired model
        messages= messages + [{"role": "user", "content": prompt}],
        temperature=0.5  # Adjust temperature as needed
    )

    # Get the generated text from the response
    generated_text = response.choices[0].message.content
    return generated_text

# Main pipeline
def run_pipeline():
    filename = "command.wav"
    
    # Step 1: Record audio from microphone
    record_audio(filename=filename)
    
    # Step 2: Encode the recorded audio in base64
    encoded_audio = encode_audio_base64(filename=filename)
    
    # Step 3: Transcribe the audio using OpenAI's Realtime Console
    transcription = transcribe_audio_realtime(encoded_audio)
    
    # Step 4: Parse the transcribed text using GPT-3.5 with Langchain tracing
    if transcription:
        parsed_command = parse_command(transcription)
        print(f"Parsed Command: {parsed_command}")
    else:
        print("No transcription available.")

# Run the full pipeline
if __name__ == "__main__":
    run_pipeline()
