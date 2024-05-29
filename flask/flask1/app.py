from flask import Flask, request, jsonify, render_template
from flask_socketio import SocketIO, emit
import rospy
from std_msgs.msg import Int32

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

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
    return render_template('index.html')

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

def ros_callback(msg):
    rospy.loginfo(f"Received from /step_control: {msg.data}")
    # 클라이언트로 메시지 전송
    socketio.emit('ros_message', {'data': msg.data})

def ros_listener():
    rospy.Subscriber('/step_control', Int32, ros_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('web_speech_node', anonymous=True)
    pub = rospy.Publisher('/step_control', Int32, queue_size=10)
    
    # 별도의 스레드에서 ROS 리스너 실행
    import threading
    thread = threading.Thread(target=ros_listener)
    thread.start()

    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
