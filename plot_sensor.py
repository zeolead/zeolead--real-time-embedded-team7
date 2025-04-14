import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv

# 创建多个图像窗口
fig_pitch, ax_pitch = plt.subplots()
fig_ay, ax_ay = plt.subplots()
fig_accel_y, ax_accel_y = plt.subplots()
fig_accel_z, ax_accel_z = plt.subplots()

# 存储数据
pitch_data = []
ay_data = []
accel_y_data = []
accel_z_data = []

# 读取并解析数据
def read_data():
    global pitch_data, ay_data, accel_y_data, accel_z_data
    try:
        with open('sensor_data.csv', 'r') as f:
            lines = f.readlines()
            lines = lines[-100:]  # 保留最近100行
            pitch_data.clear()
            ay_data.clear()
            accel_y_data.clear()
            accel_z_data.clear()
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) >= 4:
                    pitch_data.append(float(parts[0]))
                    ay_data.append(float(parts[1]))
                    accel_y_data.append(float(parts[2]))
                    accel_z_data.append(float(parts[3]))
    except FileNotFoundError:
        pass

# 各个动画更新函数
def animate_pitch(i):
    read_data()
    ax_pitch.clear()
    ax_pitch.plot(pitch_data, label='Pitch (°)', color='b')
    ax_pitch.set_title("Pitch Angle")
    ax_pitch.set_ylim(-90, 90)
    ax_pitch.legend()

def animate_ay(i):
    ax_ay.clear()
    ax_ay.plot(ay_data, label='ay (g)', color='g')
    ax_ay.set_title("Forward/Backward Acceleration (ay)")
    ax_ay.set_ylim(-2, 2)
    ax_ay.legend()

def animate_accel_y(i):
    ax_accel_y.clear()
    ax_accel_y.plot(accel_y_data, label='accel_y raw (g)', color='orange')
    ax_accel_y.set_title("Raw Accel Y")
    ax_accel_y.set_ylim(-2, 2)
    ax_accel_y.legend()

def animate_accel_z(i):
    ax_accel_z.clear()
    ax_accel_z.plot(accel_z_data, label='accel_z raw (g)', color='red')
    ax_accel_z.set_title("Raw Accel Z")
    ax_accel_z.set_ylim(-2, 2)
    ax_accel_z.legend()

# 分别为每个窗口启动动画
ani_pitch = animation.FuncAnimation(fig_pitch, animate_pitch, interval=100)
ani_ay = animation.FuncAnimation(fig_ay, animate_ay, interval=100)
ani_accel_y = animation.FuncAnimation(fig_accel_y, animate_accel_y, interval=100)
ani_accel_z = animation.FuncAnimation(fig_accel_z, animate_accel_z, interval=100)

# 显示所有图像窗口
plt.show()
