import streamlit as st
from PIL import Image

st.title("골프 자세 분석")

#recoding_video_path = '/home/sang/catkin_ws/src/Adeept-RaspberryYOLO-Follower/src/myagv_controller/video/1.mp4 '
#anlayze_video_capture = '/home/sang/catkin_ws/src/Adeept-RaspberryYOLO-Follower/src/myagv_controller/video/1_captured_frame.jpg'
#anlayze_result_video ='/home/sang/catkin_ws/src/Adeept-RaspberryYOLO-Follower/src/myagv_controller/video/1_output.mp4'

recoding_video_path = "/home/sang/test_js/video/1.mp4"
recoding_video_path_file= open(recoding_video_path, "rb")
recoding_video_path_bytes= recoding_video_path_file.read()

anlayze_video_capture = "/home/sang/test_js/video/1_captured_frame.jpg"
anlayze_video_capture_image_file =Image.open(anlayze_video_capture)

anlayze_result_video_path ="/home/sang/test_js/video/1_output.mp4"
anlayze_result_video_path_file = open(anlayze_video_capture, "rb")
anlayze_result_video_path_byte  =  anlayze_result_video_path_file.read()

col1, col2 ,col3 = st.columns([1,1,1])
with col1:
    st.header("녹화 영상")
    st.video(recoding_video_path_bytes)
with col2:
    st.header("주요 장면")
    st.image(anlayze_video_capture_image_file)
with col3:
    st.header("분석 결과")
    st.video(anlayze_result_video_path_byte)
# comment analyze 

content = st.text_area("피드백: ", "")

if st.button("이전"):
    st.switch_page('gui.py')