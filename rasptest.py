from flask import Flask, request, Response, jsonify #Tạo web server - Nhận dữ liệu lệnh điều khiển gửi từ client - Trả dữ liệu video dạng stream
import cv2 #OpenCV để xử lý và lấy hình ảnh từ camera
import RPi.GPIO as GPIO #Điều khiển GPIO trên Raspberry Pi
from flask import Response, stream_with_context
from flask_cors import CORS
import json
import time
import board
import busio
import adafruit_bmp280
from adafruit_pca9685 import PCA9685
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from simple_pid import PID
import threading
import math

app = Flask(__name__)
CORS(app)

i2c = busio.I2C(board.SCL, board.SDA)

# MPU9250 (IMU)
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,
    bus=1,
    gfs=GFS_250,  # Gyroscope full scale
    afs=AFS_2G,   # Accelerometer full scale
    mfs=AK8963_BIT_16,  # Magnetometer resolution
    mode=AK8963_MODE_C100HZ
)
try:
    mpu.configure()  # Apply settings
except Exception as e:
    print("Không thể cấu hình từ kế, bỏ qua magnetometer:", e)

# BMP280 (áp suất & nhiệt độ)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
bmp280.sea_level_pressure = 1013.25  # Điều chỉnh theo vị trí thực

#PID
pitch = 0.0
roll = 0.0
yaw = 0.0
last_time = time.time()
alpha = 0.98  # Hệ số Complementary Filter (tinh chỉnh nếu cần)

Kp_yaw = Kp_pitch = Kp_roll = 1.0
Ki_yaw = Ki_pitch = Ki_roll = 0.0
Kd_yaw = Kd_pitch = Kd_roll = 0.1

prev_error = {'yaw': 0, 'pitch': 0, 'roll': 0}
integral_error = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
target_values = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'Dosau': 0.0}

current_depth = 0.0
pid = PID(10, 0.5, 2, setpoint=target_values['Dosau'])
pid.output_limits = (0, 65535)

pca = PCA9685(i2c) #Tạo đối tượng PCA9685
pca.frequency = 1000 #Tần số cho động cơ 

# Gán các kênh băm xung cho động cơ trên mạch PCA9685
LPWM1 = pca.channels[0]
RPWM1 = pca.channels[1]
LPWM2 = pca.channels[2]
RPWM2 = pca.channels[3]
LPWM3 = pca.channels[4]
RPWM3 = pca.channels[5]
LPWM4 = pca.channels[6]
RPWM4 = pca.channels[7]
#Gắn các kênh cho động cơ bơm lấy mẫu nước
in1 = pca.channels[8]
in2 = pca.channels[9]
in3 = pca.channels[10]
in4 = pca.channels[11]


# Thiết lập GPIO điều khiển động cơ
#GPIO 04 = in1_L298_1
#GPIO 18 = in2_L298_1
#GPIO 27 = in3_L298_1
#GPIO 22 = in4_L298_1

#GPIO 23 = in1_L298_2
#GPIO 24 = in2_L298_2
#GPIO 25 = in3_L298_2
#GPIO 05 = in4_L298_2

#GPIO 06 = in1_L298_3
#GPIO 12 = in2_L298_3
#GPIO 13 = in3_L298_3
#GPIO 19 = in4_L298_3

#GPIO 16 = in1_L298_4
#GPIO 26 = in2_L298_4
#GPIO 20 = in3_L298_4
#GPIO 21 = in4_L298_4

#GPIO 17 = light_on
# Thiết lập các chân GPIO làm OUTPUT
GPIO.setmode(GPIO.BCM) #Để dùng đúng cách đánh số chân.
GPIO.setwarnings(False) #Tắt cảnh báo khi sử dụng lại các chân GPIO
gpio_pins = [4, 18, 27, 22, 23, 24, 25, 5, 6, 12, 13, 16, 26, 20, 21, 17, 11]
for pin in gpio_pins:
    GPIO.setup(pin, GPIO.OUT)

mode = "web" 
pid_active = False

@app.route('/get_mode')
def get_mode():
    return jsonify({"mode": mode})

@app.route('/control', methods=['POST'])
def control():
    global mode
    data = request.get_json()   
    print(" Dữ liệu nhận:", data)

    if data.get("mode") == "set":
        mode = data.get("value", "web")
        print(" Đã chuyển chế độ sang:", mode)
        return jsonify({"status": "ok", "mode": mode})
    
    if "pid_mode" in data:
        pid_active = data["pid_mode"]
        print("PID mode is now", "ON" if pid_active else "OFF")
        return jsonify({"status": "ok", "pid_active": pid_active})
    
    cmds = data.get("cmds", [])
    if isinstance(cmds, str):
        cmds = [cmds]  # chuyển chuỗi đơn thành danh sách
    if cmds == "PID_Setpoints":
        target_values['pitch'] = float(data.get('pitch', 0.0))
        target_values['yaw'] = float(data.get('yaw', 0.0))
        target_values['roll'] = float(data.get('roll', 0.0))
        target_values['Dosau'] = float(data.get('Dosau',0.0))
        print(f" Cập nhật Target PID: pitch={target_values['pitch']}, yaw={target_values['yaw']}, roll={target_values['roll']}")
        return jsonify({"status": "ok", "target_values": target_values})
    source = "PS2" if cmds.endswith("PS2") else "Web"
    print(f" Nhận lệnh từ {source}: {cmds}")
    #Lấy pwmValue
    pwmValue = data.get("pwmValue", None)

    if pwmValue is not None:
        pwmValue = int(float(pwmValue))
        #Quy về thang 16-bit để PCA9685 hiểu
        PWMWEB = int(pwmValue * 65535 / 255)
        print(f" PWM nhận được từ slider: {pwmValue}")

        
    def parse_pwm(value):
        try:
            return int(float(value))
        except:
            return 0

    pwmLeftPS2 = parse_pwm(data.get('pwmLeftPS2', 0))
    pwmRightPS2 = parse_pwm(data.get('pwmRightPS2', 0))

    if mode == "web" and cmds.endswith("PS2"):
        print(" Web đang chạy. Bỏ qua lệnh PS2:", cmds)
        return jsonify({"status": "ignored", "reason": "web mode"})
    elif mode == "ps2" and not cmds.endswith("PS2"):
        print(" PS2 đang chạy. Bỏ qua lệnh Web:", cmds)
        return jsonify({"status": "ignored", "reason": "ps2 mode"})

    print(f" Xử lý lệnh: {cmds} (PWM: Left={pwmLeftPS2}, Right={pwmRightPS2})")
    # TODO: Gửi lệnh điều khiển động cơ tại đây
    if cmds == "Tien":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)        
        LPWM1.duty_cycle = PWMWEB
        RPWM1.duty_cycle = PWMWEB
        LPWM2.duty_cycle = PWMWEB
        RPWM2.duty_cycle = PWMWEB 
    elif cmds == "Tientrai":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH)  

        GPIO.output(23, GPIO.HIGH)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)  
        LPWM1.duty_cycle = PWMWEB
        RPWM1.duty_cycle = PWMWEB 
        LPWM2.duty_cycle = PWMWEB
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Tienphai":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.HIGH)
        GPIO.output(5, GPIO.LOW)  
        LPWM1.duty_cycle = PWMWEB 
        RPWM1.duty_cycle = PWMWEB
        LPWM2.duty_cycle = PWMWEB 
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Lui":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH)  
        LPWM1.duty_cycle = PWMWEB 
        RPWM1.duty_cycle = PWMWEB 
        LPWM2.duty_cycle = PWMWEB
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Luitrai":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.HIGH)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH) 
        LPWM1.duty_cycle = PWMWEB 
        RPWM1.duty_cycle = PWMWEB 
        LPWM2.duty_cycle = PWMWEB
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Luiphai":
        GPIO.output(4, GPIO.HIGH)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH) 
        LPWM1.duty_cycle = PWMWEB 
        RPWM1.duty_cycle = PWMWEB
        LPWM2.duty_cycle = PWMWEB
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Trai":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH)  
        LPWM1.duty_cycle = PWMWEB 
        RPWM1.duty_cycle = PWMWEB
        LPWM2.duty_cycle = PWMWEB 
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Phai":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)      
        LPWM1.duty_cycle = PWMWEB
        RPWM1.duty_cycle = PWMWEB 
        LPWM2.duty_cycle = PWMWEB
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Quaytrai":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH)
        LPWM1.duty_cycle = PWMWEB
        RPWM1.duty_cycle = PWMWEB
        LPWM2.duty_cycle = PWMWEB
        RPWM2.duty_cycle = PWMWEB 
    elif cmds == "Quayphai":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)
        LPWM1.duty_cycle = PWMWEB
        RPWM1.duty_cycle = PWMWEB 
        LPWM2.duty_cycle = PWMWEB 
        RPWM2.duty_cycle = PWMWEB
    elif cmds == "Lan":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.HIGH)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.HIGH) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.HIGH) 
        LPWM3.duty_cycle = PWMWEB
        RPWM3.duty_cycle = PWMWEB
        LPWM4.duty_cycle = PWMWEB 
        RPWM4.duty_cycle = PWMWEB
    elif cmds == "Noi":
        GPIO.output(6, GPIO.HIGH)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.HIGH)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.HIGH)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = PWMWEB 
        RPWM3.duty_cycle = PWMWEB
        LPWM4.duty_cycle = PWMWEB 
        RPWM4.duty_cycle = PWMWEB    
    elif cmds == "Nghientruoc_Down":
        GPIO.output(6, GPIO.HIGH)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = PWMWEB
        RPWM3.duty_cycle = PWMWEB 
        LPWM4.duty_cycle = PWMWEB
        RPWM4.duty_cycle = PWMWEB
    elif cmds == "Nghientruoc_Up":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.HIGH)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.HIGH) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = PWMWEB
        RPWM3.duty_cycle = PWMWEB 
        LPWM4.duty_cycle = PWMWEB
        RPWM4.duty_cycle = PWMWEB
    elif cmds == "Nghientrai_Down":
        GPIO.output(6, GPIO.HIGH)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.HIGH)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = PWMWEB 
        RPWM3.duty_cycle = PWMWEB
        LPWM4.duty_cycle = PWMWEB 
        RPWM4.duty_cycle = PWMWEB 
    elif cmds == "Nghientrai_Up":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.HIGH)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = PWMWEB 
        RPWM3.duty_cycle = PWMWEB
        LPWM4.duty_cycle = PWMWEB
        RPWM4.duty_cycle = PWMWEB
    elif cmds == "Nghienphai_Down":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.HIGH)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = PWMWEB
        RPWM3.duty_cycle = PWMWEB 
        LPWM4.duty_cycle = PWMWEB
        RPWM4.duty_cycle = PWMWEB
    elif cmds == "Nghienphai_Up":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.HIGH) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.HIGH) 
        LPWM3.duty_cycle = PWMWEB
        RPWM3.duty_cycle = PWMWEB 
        LPWM4.duty_cycle = PWMWEB
        RPWM4.duty_cycle = PWMWEB       
    elif cmds == "Nghiensau_Down":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.HIGH)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.HIGH)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = PWMWEB
        RPWM3.duty_cycle = PWMWEB
        LPWM4.duty_cycle = PWMWEB
        RPWM4.duty_cycle = PWMWEB
    elif cmds == "Nghiensau_Up":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.HIGH) 
        LPWM3.duty_cycle = PWMWEB
        RPWM3.duty_cycle = PWMWEB
        LPWM4.duty_cycle = PWMWEB
        RPWM4.duty_cycle = PWMWEB 
    elif cmds == "Batden":
        GPIO.output(11, GPIO.HIGH) 
    elif cmds == "Tatden":
        GPIO.output(11, GPIO.LOW)
    elif cmds == "Laynuoc":
        in1.duty_cycle = 0xFFFF #Kích lên hẳn mức high để cho in1 = 1, in2 = 0 -> kích bơm hút nước
        in2.duty_cycle = 0
        in3.duty_cycle = 0
        in4.duty_cycle = 0
    elif cmds == "Daynuoc":
        in1.duty_cycle = 0
        in2.duty_cycle = 0
        in3.duty_cycle = 0xFFFF #Kích lên hẳn mức high để cho in3 = 1, in4 = 0 -> kích bơm đẩy nước
        in4.duty_cycle = 0  
    elif cmds == "Dunglaynuoc":
        in1.duty_cycle = 0
        in2.duty_cycle = 0
        in3.duty_cycle = 0
        in4.duty_cycle = 0     
    if cmds == "TienPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)          
        LPWM1.duty_cycle = 0
        RPWM1.duty_cycle = 0
        LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
    elif cmds == "TientraiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH)  

        GPIO.output(23, GPIO.HIGH)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)  
        LPWM1.duty_cycle = 0
        RPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
    elif cmds == "TienphaiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.HIGH)
        GPIO.output(5, GPIO.LOW)  
        LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM1.duty_cycle = 0 
        LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
    elif cmds == "LuiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH)  
        LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        LPWM2.duty_cycle = 0
        RPWM2.duty_cycle = 0
    elif cmds == "LuitraiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.HIGH)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH) 
        LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255) 
        LPWM2.duty_cycle = 0
        RPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
    elif cmds == "LuiphaiPS2":
        GPIO.output(4, GPIO.HIGH)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH) 
        LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM2.duty_cycle = 0
    elif cmds == "TraiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH)  
        LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM1.duty_cycle = 0
        LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        RPWM2.duty_cycle = 0
    elif cmds == "PhaiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)      
        LPWM1.duty_cycle = 0
        RPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
        LPWM2.duty_cycle = 0
        RPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
    elif cmds == "QuaytraiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.HIGH) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.HIGH)
        LPWM1.duty_cycle = 0xFFFF
        RPWM1.duty_cycle = 0
        LPWM2.duty_cycle = 0
        RPWM2.duty_cycle = 0xFFFF
    elif cmds == "QuayphaiPS2":
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)
        LPWM1.duty_cycle = 0
        RPWM1.duty_cycle = 0xFFFF 
        LPWM2.duty_cycle = 0xFFFF 
        RPWM2.duty_cycle = 0 
    elif cmds == "LanPS2":
        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.HIGH)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.HIGH) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.HIGH) 
        LPWM3.duty_cycle = int(pwmRightPS2 * 65535 / 255) 
        RPWM3.duty_cycle = int(pwmRightPS2 * 65535 / 255) 
        LPWM4.duty_cycle = int(pwmRightPS2 * 65535 / 255) 
        RPWM4.duty_cycle = int(pwmRightPS2 * 65535 / 255) 
    elif cmds == "NoiPS2":
        GPIO.output(6, GPIO.HIGH)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.HIGH)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.HIGH)
        GPIO.output(21, GPIO.LOW) 
        LPWM3.duty_cycle = int(pwmRightPS2 * 65535 / 255) 
        RPWM3.duty_cycle = int(pwmRightPS2 * 65535 / 255) 
        LPWM4.duty_cycle = int(pwmRightPS2 * 65535 / 255) 
        RPWM4.duty_cycle = int(pwmRightPS2 * 65535 / 255)     
    elif cmds == "BatdenPS2":
        GPIO.output(11, GPIO.HIGH) 
    elif cmds == "TatdenPS2":
        GPIO.output(11, GPIO.LOW)
    elif cmds == "LaynuocPS2":
        in1.duty_cycle = 0xFFFF 
        in2.duty_cycle = 0
        in3.duty_cycle = 0
        in4.duty_cycle = 0
    elif cmds == "DaynuocPS2":
        in1.duty_cycle = 0
        in2.duty_cycle = 0
        in3.duty_cycle = 0xFFFF 
        in4.duty_cycle = 0  
    elif cmds == "DunglaynuocPS2":
        in1.duty_cycle = 0
        in2.duty_cycle = 0
        in3.duty_cycle = 0
        in4.duty_cycle = 0          
    elif cmds in ["Dung", "DungPS2"]:
        GPIO.output(4, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        GPIO.output(27, GPIO.LOW)
        GPIO.output(22, GPIO.LOW) 

        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)
        GPIO.output(5, GPIO.LOW)  

        GPIO.output(6, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(17, GPIO.LOW) 
        
        GPIO.output(16, GPIO.LOW)
        GPIO.output(26, GPIO.LOW)
        GPIO.output(20, GPIO.LOW)
        GPIO.output(21, GPIO.LOW) 

        LPWM1.duty_cycle = 0
        RPWM1.duty_cycle = 0
        LPWM2.duty_cycle = 0
        RPWM2.duty_cycle = 0
        LPWM3.duty_cycle = 0
        RPWM3.duty_cycle = 0
        LPWM4.duty_cycle = 0
        RPWM4.duty_cycle = 0
    return jsonify({"status": "ok"})
camera = cv2.VideoCapture(0) #Mở camera mặc định (index 0)
#Đặt độ phân giải khung hình video: 1280x720
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not camera.isOpened():
    raise RuntimeError("Không mở được camera")
def generate_video():
    while True:
        success, frame = camera.read()
        if not success:
            break
        _, buffer = cv2.imencode('.jpg', frame) #Đọc frame từ camera sau đó Mã hoá thành .jpg
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n') #Trả lại từng đoạn ảnh theo chuẩn MJPEG stream để client hiển thị liên tục

@app.route('/video_feed')
def video_feed(): #gọi hàm generate_video để stream từ camera
    return Response(generate_video(), mimetype='multipart/x-mixed-replace; boundary=frame') #mimetype='multipart/x-mixed-replace' cho phép stream nhiều ảnh liên tục như 1 video
@app.route('/sensor_stream')
def sensor_stream():
    def generate_sensor_data():
        while True:
            # Đọc dữ liệu cảm biến
            ax, ay, az = mpu.readAccelerometerMaster()
            gx, gy, gz = mpu.readGyroscopeMaster()
            temp = bmp280.temperature
            pressure = bmp280.pressure
            # Tạo dữ liệu JSON
            data = {
                "acc": {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)},
                "gyro": {"x": round(gx, 2), "y": round(gy, 2), "z": round(gz, 2)},
                "temp": round(temp, 2),
                "pressure": round(pressure, 2)
            }
            # Stream dưới dạng text/plain
            yield f"data: {json.dumps(data)}\n\n"
            time.sleep(0.2)
    return Response(stream_with_context(generate_sensor_data()), mimetype='text/event-stream')

def pid_control(axis, current, target, Kp, Ki, Kd):
    global prev_error, integral_error
    error = target - current
    integral_error[axis] += error
    max_integral = 100.0
    integral_error[axis] = max(min(integral_error[axis], max_integral), -max_integral)
    derivative = error - prev_error[axis]
    prev_error[axis] = error
    return Kp * error + Ki * integral_error[axis] + Kd * derivative

# Hàm điều khiển động cơ theo 4 trục

def apply_corrections(yaw_out, pitch_out, roll_out, depth_out):
    # Điều khiển tất cả 8 motor cùng lúc để ổn định ROV
    # Tạo mô hình động cơ:
    # M1: Front Left  (LPWM1 / GPIO 4-18)
    # M2: Front Right (RPWM1 / GPIO 27-22)
    # M3: Back Left   (LPWM2 / GPIO 23-24)
    # M4: Back Right  (RPWM2 / GPIO 25-5)
    # M5: Vert Front Left  (LPWM3 / GPIO 6-12)
    # M6: Vert Front Right (RPWM3 / GPIO 13-17)
    # M7: Vert Back Left   (LPWM4 / GPIO 16-26)
    # M8: Vert Back Right  (RPWM4 / GPIO 20-21)

    # Tổng hợp lực cho từng motor (M1 → M8):
    #     Yaw  ảnh hưởng M1–M4
    #     Pitch & Roll ảnh hưởng M5–M8

    # Lực trên mỗi motor = tổ hợp từ các trục (tùy vị trí)
    motor_forces = {
        "M1": +yaw_out,      # quay phải
        "M2": -yaw_out,      # quay trái
        "M3": -yaw_out,
        "M4": +yaw_out,
        "M5": depth_out - pitch_out - roll_out,  # FL dọc
        "M6": depth_out - pitch_out + roll_out,  # FR dọc
        "M7": depth_out + pitch_out - roll_out,  # BL dọc
        "M8": depth_out + pitch_out + roll_out,  # BR dọc
    }

    # Tính hệ số scale nếu có motor nào vượt 65535
    max_force = max(abs(v) for v in motor_forces.values())
    scale = 65535 / max_force if max_force > 65535 else 1

    #Thiết lập hàm tiện ích điều khiển motor
    def set_motor(pwm, gpio1, gpio2, pwm_pin):
        if pwm >= 0:
            GPIO.output(gpio1, GPIO.HIGH)
            GPIO.output(gpio2, GPIO.LOW)
        else:
            GPIO.output(gpio1, GPIO.LOW)
            GPIO.output(gpio2, GPIO.HIGH)
        pwm_pin.duty_cycle = int(min(abs(pwm * scale), 65535))

    # Gán từng motor theo GPIO và PWM tương ứng
    set_motor(motor_forces["M1"], 4, 18, LPWM1)
    set_motor(motor_forces["M2"], 27, 22, RPWM1)
    set_motor(motor_forces["M3"], 23, 24, LPWM2)
    set_motor(motor_forces["M4"], 25, 5, RPWM2)
    set_motor(motor_forces["M5"], 6, 12, LPWM3)
    set_motor(motor_forces["M6"], 13, 17, RPWM3)
    set_motor(motor_forces["M7"], 16, 26, LPWM4)
    set_motor(motor_forces["M8"], 20, 21, RPWM4)
    # Tổng hợp lực từ 3 trục: depth, pitch, roll để điều khiển 4 motor dọc
    # motor_FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    motor_FL = depth_out - pitch_out - roll_out
    motor_FR = depth_out - pitch_out + roll_out
    motor_BL = depth_out + pitch_out - roll_out
    motor_BR = depth_out + pitch_out + roll_out

    motors = [motor_FL, motor_FR, motor_BL, motor_BR]
    max_pwm = max(abs(m) for m in motors)
    scale = 65535 / max_pwm if max_pwm > 65535 else 1

    pwm_FL = int(min(abs(motor_FL * scale), 65535))
    pwm_FR = int(min(abs(motor_FR * scale), 65535))
    pwm_BL = int(min(abs(motor_BL * scale), 65535))
    pwm_BR = int(min(abs(motor_BR * scale), 65535))

    # Thiết lập hướng quay theo dấu
    def set_motor_direction(pwm, direction_gpio_1, direction_gpio_2, duty_pin):
        if pwm >= 0:
            GPIO.output(direction_gpio_1, GPIO.HIGH)
            GPIO.output(direction_gpio_2, GPIO.LOW)
        else:
            GPIO.output(direction_gpio_1, GPIO.LOW)
            GPIO.output(direction_gpio_2, GPIO.HIGH)
        duty_pin.duty_cycle = abs(pwm)

    # Mapping GPIO tùy chỉnh theo từng motor
    # Ví dụ: FL: GPIO6/12 → LPWM3, FR: GPIO13/17 → RPWM3, BL: GPIO16/26 → LPWM4, BR: GPIO20/21 → RPWM4
    set_motor_direction(motor_FL, 6, 12, LPWM3)
    set_motor_direction(motor_FR, 13, 17, RPWM3)
    set_motor_direction(motor_BL, 16, 26, LPWM4)
    set_motor_direction(motor_BR, 20, 21, RPWM4)

    # Điều khiển YAW bằng 4 động cơ ngang (giữ nguyên)
    if abs(yaw_out) > 1:
        duty_yaw = int(min(abs(yaw_out), 65535))
        if yaw_out > 0:
            GPIO.output(4, GPIO.LOW)
            GPIO.output(18, GPIO.HIGH)
            GPIO.output(27, GPIO.LOW)
            GPIO.output(22, GPIO.LOW)

            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)
            GPIO.output(25, GPIO.LOW)
            GPIO.output(5, GPIO.LOW)

            LPWM1.duty_cycle = 0
            RPWM1.duty_cycle = duty_yaw
            LPWM2.duty_cycle = duty_yaw
            RPWM2.duty_cycle = 0
        else:
            GPIO.output(4, GPIO.LOW)
            GPIO.output(18, GPIO.LOW)
            GPIO.output(27, GPIO.LOW)
            GPIO.output(22, GPIO.HIGH)

            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.LOW)
            GPIO.output(25, GPIO.LOW)
            GPIO.output(5, GPIO.HIGH)

            LPWM1.duty_cycle = duty_yaw
            RPWM1.duty_cycle = 0
            LPWM2.duty_cycle = 0
            RPWM2.duty_cycle = duty_yaw

def pid_loop():
    global pitch, roll, yaw, current_depth
    while True:
        if pid_active:
            try:
                ax, ay, az = mpu.readAccelerometerMaster()
                gx, gy, gz = mpu.readGyroscopeMaster()
                pressure = bmp280.pressure
                current_depth = (pressure - 1013.25) / 10.0

                dt = 0.05
                acc_pitch = math.degrees(math.atan2(ay, az))
                acc_roll = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

                gyro_pitch = gx * dt
                gyro_roll = gy * dt
                gyro_yaw = gz * dt

                pitch = alpha * (pitch + gyro_pitch) + (1 - alpha) * acc_pitch
                roll = alpha * (roll + gyro_roll) + (1 - alpha) * acc_roll
                yaw += gyro_yaw

                out_pitch = pid_control("pitch", pitch, target_values["pitch"], Kp_pitch, Ki_pitch, Kd_pitch)
                out_roll = pid_control("roll", roll, target_values["roll"], Kp_roll, Ki_roll, Kd_roll)
                out_yaw = pid_control("yaw", yaw, target_values["yaw"], Kp_yaw, Ki_yaw, Kd_yaw)
                out_depth = pid(current_depth)

                print(f"PID ➤ Pitch: {out_pitch:.2f}, Roll: {out_roll:.2f}, Yaw: {out_yaw:.2f}, Depth: {out_depth:.2f}")
                apply_corrections(out_yaw, out_pitch, out_roll, out_depth)

            except Exception as e:
                print("Lỗi PID loop:", e)

        time.sleep(0.05)


@app.route('/orientation_stream')
def orientation_stream():
    def generate_orientation_data():
        while True:
            data = {
                "pitch": round(pitch, 2),
                "roll": round(roll, 2),
                "yaw": round(yaw, 2),
            }
            yield f"data: {json.dumps(data)}\n\n"
            time.sleep(0.1)
    return Response(generate_orientation_data(), mimetype='text/event-stream')

threading.Thread(target=pid_loop, daemon=True).start()

app.run(host='0.0.0.0', port=5000)