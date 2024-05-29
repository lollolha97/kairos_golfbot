import requests
import json

# OpenWeatherMap API 설정
apikey = '95bfb436398361dce18ecda454fe5100'
city = 'Seoul'
lang = 'kr'
api = f"""http://api.openweathermap.org/data/2.5/weather?q={city}&appid={apikey}&lang={lang}&units=metric"""
result = requests.get(api)
data = json.loads(result.text)

print(data)