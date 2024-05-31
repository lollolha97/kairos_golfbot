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
def pub_message_ros(client,data):
    pub = roslibpy.Topic(client, '/step_control', 'std_msgs/Int32')
    pub.publish(roslibpy.Message({'data': data}))

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
    st.title(" :golf: 스마트 캐디봇 :golfer: ")
    # st.image('/home/sang/test_js/gif/logo.png', caption='KG Kairos Gasan Center C401-2', use_column_width=True)
    client = roslibpy.Ros(host='localhost', port=9090)
    listener = roslibpy.Topic(client, '/step_str', 'std_msgs/String')
    listener.subscribe(callback) 
    client.run()

    placeholder  = st.empty()
    loading_bar = st.empty()

    if st.button('이동',use_container_width=True):
        st.session_state.button_clicked1 = True
        if st.session_state.button_clicked1:
            pub_message_ros(client, 0)
            st.session_state.button_clicked1 = False
    if st.button('정지',use_container_width=True):
        st.session_state.button_clicked = True
        if st.session_state.button_clicked:
            pub_message_ros(client, 100)
            st.session_state.button_clicked = False

    if message_queue.get() == "분석중입니다...":
        bar = st.progress(0)
        for i in range(100):
            loading_bar.text (f'영상 분석중 : {i+1}%')
            bar.progress(i+1)
            time.sleep(0.05)
            if i == 99:
                st.switch_page('pages/page_1.py')
    
    if st.button("분석 결과",use_container_width=True):
        st.switch_page('pages/page_1.py')
    
    if data["cod"] == 200:
        html_code = f"""
        <div style="background-color: #B0E0E6; padding: 20px; border-radius: 10px;">
            <h1 style="text-align: center;">Weather in {city}</h1>
            <div style="display: flex; align-items: center; justify-content: space-around;">
                <img src="{icon_url}" alt="Weather Icon" style="width: 100px; height: 100px;">
                <div>
                    <p style="font-size: 20px; margin: 5px 0;">날씨: {data['weather'][0]['description']}</p>
                    <p style="font-size: 20px; margin: 5px 0;">온도: {data['main']['temp']} ℃</p>
                    <p style="font-size: 20px; margin: 5px 0;">풍속: {data['wind']['speed']} m/s</p>
                    <p style="font-size: 20px; margin: 5px 0;">풍향: {data['wind']['deg']}°</p>
                </div>
            </div>
        </div>
        """
        st.write(html_code, unsafe_allow_html=True)
    else:
        st.error(f"Failed to retrieve weather data: {data['message']}")

    # Main loop to update the Streamlit UI with received messages   
    while True:
        if not message_queue.empty():
            message = message_queue.get()
            global previous_message
            if message != previous_message:
                placeholder.text(f"{message}")
                previous_message = message

if __name__ == "__main__":
    main()


