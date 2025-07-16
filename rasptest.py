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
alpha = 0.9  # Hệ số Complementary Filter (tinh chỉnh nếu cần)

Kp_yaw = Kp_pitch = Kp_roll = 1.0
Ki_yaw = Ki_pitch = Ki_roll = 0.01
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
LPWM5 = pca.channels[8]
RPWM5 = pca.channels[9]
LPWM6 = pca.channels[10]
RPWM6 = pca.channels[11]
LPWM7 = pca.channels[12]
RPWM7 = pca.channels[13]
LPWM8 = pca.channels[14]
RPWM8 = pca.channels[15]

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
    global Dung
    data = request.get_json()   
    print(" Dữ liệu nhận:", data)

    if data.get("mode") == "set":
        mode = data.get("value", "web")
        print(" Đã chuyển chế độ sang:", mode)
        return jsonify({"status": "ok", "mode": mode})
    
    if "pid_mode" in data:
        global pid_active 
        pid_active = data["pid_mode"]
        print("PID mode is now", "ON" if pid_active else "OFF")
        return jsonify({"status": "ok", "pid_active": pid_active})
    
    cmds = data.get("cmds")
    if isinstance(cmds, str):
        cmds = [cmds]  # chuyển chuỗi đơn thành danh sách
    if cmds == ["PID_Setpoints"] or (len(cmds) == 1 and cmds[0] == "PID_Setpoints"):
        target_values['pitch'] = float(data.get('pitch', 0.0))
        target_values['yaw'] = float(data.get('yaw', 0.0))
        target_values['roll'] = float(data.get('roll', 0.0))
        target_values['Dosau'] = float(data.get('Dosau',0.0))
        print(f" Cập nhật Target PID: pitch={target_values['pitch']}, yaw={target_values['yaw']}, roll={target_values['roll']}")
        return jsonify({"status": "ok", "target_values": target_values})
    if isinstance(cmds, list):
        source = "PS2" if all(c.endswith("PS2") for c in cmds) else "Web"
    else:
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

    if mode == "web" and any(c.endswith("PS2") for c in cmds):
        print(" Web đang chạy. Bỏ qua lệnh PS2:", cmds)
        return jsonify({"status": "ignored", "reason": "web mode"})
    elif mode == "ps2" and all(not c.endswith("PS2") for c in cmds):
        print(" PS2 đang chạy. Bỏ qua lệnh Web:", cmds)
        return jsonify({"status": "ignored", "reason": "ps2 mode"})

    print(f" Xử lý lệnh: {cmds} (PWM: Left={pwmLeftPS2}, Right={pwmRightPS2})")
    # TODO: Gửi lệnh điều khiển động cơ tại đây
    # Nếu cmds là danh sách, lấy phần tử đầu tiên
    for cmd in cmds:
        if cmd == "Tien":
            GPIO.output(4, GPIO.HIGH)
            LPWM3.duty_cycle = PWMWEB
            RPWM3.duty_cycle = 0
            LPWM4.duty_cycle = PWMWEB
            RPWM4.duty_cycle = 0
        elif cmd == "Tientrai":
            GPIO.output(4, GPIO.HIGH)
            LPWM2.duty_cycle = 0
            RPWM2.duty_cycle = PWMWEB
            LPWM3.duty_cycle = PWMWEB
            RPWM3.duty_cycle = 0
            LPWM4.duty_cycle = PWMWEB
            RPWM4.duty_cycle = 0
        elif cmd == "Tienphai":
            GPIO.output(4, GPIO.HIGH)
            LPWM1.duty_cycle = 0
            RPWM1.duty_cycle = PWMWEB
            LPWM3.duty_cycle = PWMWEB
            RPWM3.duty_cycle = 0
            LPWM4.duty_cycle = PWMWEB
            RPWM4.duty_cycle = 0
        elif cmd == "Lui":
            GPIO.output(4, GPIO.HIGH) 
            LPWM1.duty_cycle = PWMWEB 
            RPWM1.duty_cycle = 0 
            LPWM2.duty_cycle = PWMWEB
            RPWM2.duty_cycle = 0
        elif cmd == "Luitrai":
            GPIO.output(4, GPIO.HIGH)
            LPWM4.duty_cycle = 0
            RPWM4.duty_cycle = PWMWEB
            LPWM1.duty_cycle = PWMWEB 
            RPWM1.duty_cycle = 0 
            LPWM2.duty_cycle = PWMWEB
            RPWM2.duty_cycle = 0
        elif cmd == "Luiphai":
            GPIO.output(4, GPIO.HIGH)
            LPWM3.duty_cycle = 0
            RPWM3.duty_cycle = PWMWEB
            LPWM1.duty_cycle = PWMWEB 
            RPWM1.duty_cycle = 0
            LPWM2.duty_cycle = PWMWEB
            RPWM2.duty_cycle = 0
        elif cmd == "Trai":
            GPIO.output(4, GPIO.HIGH)
            LPWM1.duty_cycle = PWMWEB
            RPWM1.duty_cycle = 0
            LPWM3.duty_cycle = PWMWEB
            RPWM3.duty_cycle = 0
        elif cmd == "Phai":
            GPIO.output(4, GPIO.HIGH)
            LPWM2.duty_cycle = PWMWEB
            RPWM2.duty_cycle = 0
            LPWM4.duty_cycle = PWMWEB
            RPWM4.duty_cycle = 0
        elif cmd == "Quaytrai":
            GPIO.output(4, GPIO.HIGH)
            LPWM1.duty_cycle = PWMWEB
            RPWM1.duty_cycle = 0
            LPWM4.duty_cycle = PWMWEB
            RPWM4.duty_cycle = 0
        elif cmd == "Quayphai":
            GPIO.output(4, GPIO.HIGH)
            LPWM2.duty_cycle = PWMWEB
            RPWM2.duty_cycle = 0
            LPWM3.duty_cycle = PWMWEB
            RPWM3.duty_cycle = 0
        elif cmd == "Lan":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = PWMWEB
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = PWMWEB
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = PWMWEB
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = PWMWEB
            RPWM8.duty_cycle = 0
        elif cmd == "Noi":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = PWMWEB
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = PWMWEB
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = PWMWEB
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = PWMWEB
        elif cmd == "Nghientruoc_Down":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = PWMWEB
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = PWMWEB
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = 0
        elif cmd == "Nghientruoc_Up":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = PWMWEB
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = PWMWEB
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = 0
        elif cmd == "Nghientrai_Down":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = PWMWEB
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = PWMWEB
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = 0
        elif cmd == "Nghientrai_Up":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = PWMWEB
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = PWMWEB
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = 0
        elif cmd == "Nghienphai_Down":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = PWMWEB
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = PWMWEB
            RPWM8.duty_cycle = 0
        elif cmd == "Nghienphai_Up":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = PWMWEB
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = PWMWEB    
        elif cmd == "Nghiensau_Down":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = PWMWEB
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = PWMWEB
            RPWM8.duty_cycle = 0
        elif cmd == "Nghiensau_Up":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = PWMWEB
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = PWMWEB
        elif cmd == "Batden":
            GPIO.output(11, GPIO.HIGH) 
        elif cmd == "Tatden":
            GPIO.output(11, GPIO.LOW)
        elif cmd == "Laynuoc":
            GPIO.output(18, GPIO.HIGH)
        elif cmd == "Dunglaynuoc":
            GPIO.output(18, GPIO.LOW)
        if cmd == "TienPS2":
            GPIO.output(4, GPIO.HIGH)           
            LPWM3.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM3.duty_cycle = 0
            LPWM4.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM4.duty_cycle = 0
        elif cmd == "TientraiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM2.duty_cycle = 0
            RPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            LPWM3.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM3.duty_cycle = 0
            LPWM4.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM4.duty_cycle = 0
        elif cmd == "TienphaiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM1.duty_cycle = 0
            RPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            LPWM3.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM3.duty_cycle = 0
            LPWM4.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM4.duty_cycle = 0
        elif cmd == "LuiPS2":
            GPIO.output(4, GPIO.HIGH) 
            LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255) 
            RPWM1.duty_cycle = 0 
            LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM2.duty_cycle = 0
        elif cmd == "LuitraiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM4.duty_cycle = 0
            RPWM4.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255) 
            RPWM1.duty_cycle = 0 
            LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM2.duty_cycle = 0
        elif cmd == "LuiphaiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM3.duty_cycle = 0
            RPWM3.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255) 
            RPWM1.duty_cycle = 0
            LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM2.duty_cycle = 0
            RPWM2.duty_cycle = 0
        elif cmd == "TraiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM1.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM1.duty_cycle = 0
            LPWM3.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM3.duty_cycle = 0
            RPWM2.duty_cycle = 0
        elif cmd == "PhaiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM2.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM2.duty_cycle = 0
            LPWM4.duty_cycle = int(pwmLeftPS2 * 65535 / 255)
            RPWM4.duty_cycle = 0
        elif cmd == "QuaytraiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM1.duty_cycle = 0xFFFF
            RPWM1.duty_cycle = 0
            LPWM4.duty_cycle = 0xFFFF
            RPWM4.duty_cycle = 0
        elif cmd == "QuayphaiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM2.duty_cycle = 0xFFFF
            RPWM2.duty_cycle = 0
            LPWM3.duty_cycle = 0xFFFF
            RPWM3.duty_cycle = 0
        elif cmd == "LanPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = int(pwmRightPS2 * 65535 / 255)
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = int(pwmRightPS2 * 65535 / 255)
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = int(pwmRightPS2 * 65535 / 255)
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = int(pwmRightPS2 * 65535 / 255)
            RPWM8.duty_cycle = 0
        elif cmd == "NoiPS2":
            GPIO.output(4, GPIO.HIGH)
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = int(pwmRightPS2 * 65535 / 255)
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = int(pwmRightPS2 * 65535 / 255)
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = int(pwmRightPS2 * 65535 / 255)
            LPWM8.duty_cycle = 0
            RPWM8.duty_cycle = int(pwmRightPS2 * 65535 / 255)
        elif cmd == "BatdenPS2":
            GPIO.output(11, GPIO.HIGH) 
        elif cmd == "TatdenPS2":
            GPIO.output(11, GPIO.LOW)
        elif cmd == "LaynuocPS2":
            GPIO.output(18, GPIO.HIGH)       
        elif cmd == "DunglaynuocPS2":
            GPIO.output(18, GPIO.LOW)        
        elif cmd in ["Dung", "DungPS2"]:
            GPIO.output(4, GPIO.LOW) 
            LPWM1.duty_cycle = 0
            RPWM1.duty_cycle = 0
            LPWM2.duty_cycle = 0
            RPWM2.duty_cycle = 0
            LPWM3.duty_cycle = 0
            RPWM3.duty_cycle = 0
            LPWM4.duty_cycle = 0
            RPWM4.duty_cycle = 0
            LPWM5.duty_cycle = 0
            RPWM5.duty_cycle = 0
            LPWM6.duty_cycle = 0
            RPWM6.duty_cycle = 0
            LPWM7.duty_cycle = 0
            RPWM7.duty_cycle = 0
            LPWM8.duty_cycle = 0
            RPWM8duty_cycle = 0
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
            temp = bmp280.temperature
            pressure = bmp280.pressure
            # Tạo dữ liệu JSON
            data = {
                "acc": {"x": round(ax, 2), "y": round(ay, 2), "z": round(az, 2)},
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

def apply_corrections(out_yaw, out_pitch, out_roll, out_depth):   
        if out_pitch > 0  and Dung == 1 and  abs(pitch - target_values["pitch"]) > 2:
            GPIO.output(6, GPIO.HIGH)
            GPIO.output(12, GPIO.LOW)
            GPIO.output(13, GPIO.HIGH)
            GPIO.output(17, GPIO.LOW) 
            
            GPIO.output(16, GPIO.LOW)
            GPIO.output(26, GPIO.HIGH)
            GPIO.output(20, GPIO.LOW)
            GPIO.output(21, GPIO.HIGH) 

            LPWM1.duty_cycle = int(abs(out_pitch) * 65535 / 255)
            RPWM1.duty_cycle = int(abs(out_pitch) * 65535 / 255)
            LPWM2.duty_cycle = int(abs(out_pitch) * 65535 / 255)
            RPWM2.duty_cycle = int(abs(out_pitch) * 65535 / 255)
        elif out_pitch < 0 and Dung == 1 and  abs(pitch - target_values["pitch"]) > 2:
            GPIO.output(6, GPIO.LOW)
            GPIO.output(12, GPIO.HIGH)
            GPIO.output(13, GPIO.LOW)
            GPIO.output(17, GPIO.HIGH) 
            
            GPIO.output(16, GPIO.HIGH)
            GPIO.output(26, GPIO.LOW)
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(21, GPIO.LOW) 

            LPWM1.duty_cycle = int(abs(out_pitch) * 65535 / 255)
            RPWM1.duty_cycle = int(abs(out_pitch) * 65535 / 255)
            LPWM2.duty_cycle = int(abs(out_pitch) * 65535 / 255)
            RPWM2.duty_cycle = int(abs(out_pitch) * 65535 / 255)
            
def pid_loop():
    global pitch, roll, yaw, current_depth
    global out_pitch
    while True:
        if pid_active:
            try:
                ax, ay, az = mpu.readAccelerometerMaster()
                pressure = bmp280.pressure
                current_depth = (pressure - 1013.25) / 10.0

                acc_roll = math.degrees(math.atan2(-ay, math.sqrt(ax**2 + az**2)))
                acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

                pitch =  alpha * pitch + (1 - alpha) * acc_pitch
                roll = alpha * roll + (1 - alpha) * acc_roll

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