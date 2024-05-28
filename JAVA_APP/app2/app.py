from flask import Flask, request, send_from_directory, jsonify
import rospy
from std_msgs.msg import Int32

app = Flask(__name__, static_url_path='', static_folder='.')

rospy.init_node('web_stt_publisher', anonymous=True)
pub = rospy.Publisher('/step_control', Int32, queue_size=10)

current_step = Int32()

@app.route('/stt', methods=['POST'])
def stt():
    data = request.get_json()
    text = data.get('text')
    rospy.loginfo(f"Received text: {text}")

    step_number = process_text(text)
    if step_number is not None:
        msg = Int32()
        msg.data = step_number
        pub.publish(msg)
        rospy.loginfo(f"Published message: {msg.data}")
    return 'OK', 200

def process_text(text):
    if "스텝" in text:
        words = text.split()
        for i in range(len(words)):
            if words[i] == "스텝" and i+1 < len(words):
                try:
                    return int(words[i+1])
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

def ros_callback(data):
    global current_step
    rospy.loginfo(f"Received from /step_control: {data.data}")
    current_step = data

rospy.Subscriber('/step_control', Int32, ros_callback)

@app.route('/step_data', methods=['GET'])
def get_step_data():
    return jsonify({'data': current_step.data})

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
