import streamlit as st
import roslibpy
import queue
import time
import requests
import json


message_queue = queue.Queue()
global previous_message
previous_message = None



def callback(message):
    message_queue.put(message['data'])
def pub_massge_ros(client,date):
    pub = roslibpy.Topic(client, '/step_control', 'std_msgs/Int32')
    pub.publish(roslibpy.Message({'data': date}))


city = "Seoul"
apikey = "c1f6c98d79482985f66c26c037fe666a"
lang = "kr"
api = f"https://api.openweathermap.org/data/2.5/weather?q={city}&appid={apikey}&lang={lang}&units=metric"
result = requests.get(api)
data = json.loads(result.text)

# 날씨 아이콘 URL 생성
icon_id = data["weather"][0]["icon"]
icon_url = f"http://openweathermap.org/img/wn/{icon_id}@2x.png"

def main():
    #st.title("Streamlit ROS")
    #st.image('tilte_img')    50in <
    client = roslibpy.Ros(host='172.30.1.86', port=9090)
    listener = roslibpy.Topic(client, '/step_str', 'std_msgs/String')
    listener.subscribe(callback) 
    client.run()
    
    if data["cod"] == 200:
        with st.container():
            st.title(f"Weather in {city}")
            st.subheader("날씨 정보:")
            st.image(icon_url, width=150)
            st.write(f"날씨: {data['weather'][0]['description']}")
            st.write(f"온도: {data['main']['temp']} ℃")
            st.write(f"풍속: {data['wind']['speed']} m/s, 풍향: {data['wind']['deg']}°")
    placeholder  = st.empty()
    loading_bar = st.empty()
    # button
    if "Publish to stop_state" not in st.session_state:
        st.session_state.button_clicked = False

    if  "Publish to start_state" not in st.session_state: 
        st.session_state.button_clicked1 = False

    if "recoding" not in st.session_state: 
        st.session_state.button_clicked2 = False

    if "analyze_pose_video" not in st.session_state: 
        st.session_state.button_clicked3 = False

    col1, col2 ,col3, col4 = st.columns([1,1,1,1])
    with col1:
        if st.button('stop_agv'):
            st.session_state.button_clicked = True
            if st.session_state.button_clicked:
                pub_massge_ros(client, 100)
                st.session_state.button_clicked = False
    with col2:
        if st.button('moving_agv'):
            st.session_state.button_clicked1 = True
            if st.session_state.button_clicked1:
                pub_massge_ros(client, 0)
                st.session_state.button_clicked1 = False
    with col3:    
        if st.button('recoding'):
            st.session_state.button_clicked2 = True
            if st.session_state.button_clicked2:
                pub_massge_ros(client, 30)
                st.session_state.button_clicked2 = False 
    with col4:
        if st.button('analyze_pose_page'):
            st.session_state.button_clicked3 = True
            if st.session_state.button_clicked3:
                pub_massge_ros(client, 60)
                st.session_state.button_clicked3 = False 

    if message_queue.get() == "analyze_pose_video":
        bar = st.progress(0)
        for i in range(100):
            loading_bar.text (f'analyze_pose_video : {i+1}%')
            bar.progress(i+1)
            time.sleep(0.1)
            if i == 99:
                st.switch_page('pages/page_1.py')
    
    if st.button("analyze_pose_video_page"):
        st.switch_page('pages/page_1.py')
      
    # Main loop to update the Streamlit UI with received messages   
    while True:
        if not message_queue.empty():
            message = message_queue.get()
            global previous_message
            if message != previous_message:
                placeholder.text(f"Received: {message}")
                previous_message = message


if __name__ == "__main__":
    main()


