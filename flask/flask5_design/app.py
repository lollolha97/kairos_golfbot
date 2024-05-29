from flask import Flask, request, jsonify, render_template
from flask_socketio import SocketIO, emit
import requests
import rospy
from std_msgs.msg import Int32
import threading
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

current_status = None  # 전역 변수로 현재 상태 저장

# OpenWeatherMap API 설정
API_KEY = '95bfb436398361dce18ecda454fe5100'
CITY = 'Seoul'
LANG = 'kr'
WEATHER_URL = f'http://api.openweathermap.org/data/2.5/weather?q={CITY}&appid={API_KEY}&lang={LANG}&units=metric'

def get_weather():
    try:
        response = requests.get(WEATHER_URL)
        data = json.loads(response.text)
        if data['cod'] == 200:
            weather = {
                'temperature': data['main']['temp'],
                'description': data['weather'][0]['description'],
                'icon': data['weather'][0]['icon']
            }
            return weather
        else:
            print(f"Failed to get weather data: {data['message']}")
            return None
    except Exception as e:
        rospy.logwarn(f"Failed to get weather data: {e}")
        return None

def process_speech_text(text):
    if "스텝" in text:
        words = text.split()
        for i in range(len(words)):
            if words[i] == "스텝" and i + 1 < len(words):
                try:
                    return int(words[i + 1])
                except ValueError:
                    rospy.logwarn("스텝 다음에 숫자가 없습니다.")
    elif "정지" in text:
        return 100
    elif "따라와" in text:
        return 10
    elif "녹화" in text:
        return 50
    elif "분석" in text:
        return 60
    return None

@app.route('/')
def index():
    weather = get_weather()
    return render_template('index.html', status=current_status, weather=weather)

@app.route('/video')
def video():
    return render_template('video.html')

@app.route('/get_status', methods=['GET'])
def get_status():
    global current_status
    return jsonify(status=current_status)

@app.route('/process_speech', methods=['POST'])
def process_speech():
    data = request.get_json()
    speech_text = data['speech']
    rospy.loginfo(f"Received speech: {speech_text}")

    speech_int = process_speech_text(speech_text)
    if speech_int is not None:
        # ROS 메시지 퍼블리시
        msg = Int32()
        msg.data = speech_int
        pub.publish(msg)
        return jsonify(success=True)
    else:
        return jsonify(success=False, message="유효한 명령어가 없습니다.")

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('ros_message', {'data': 'Connected to server'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

def ros_callback(msg):
    global current_status
    rospy.loginfo(f"Received from /step_control: {msg.data}")
    current_status = msg.data
    print(f"Updated current status: {current_status}")

def ros_listener():
    rospy.Subscriber('/step_control', Int32, ros_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('web_speech_node', anonymous=True)
    pub = rospy.Publisher('/step_control', Int32, queue_size=10)
    
    # 별도의 스레드에서 ROS 리스너 실행
    thread = threading.Thread(target=ros_listener)
    thread.start()

    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
