# test_api.py
from machine import PWM
from media.sensor import *
import image

print("=== 测试 PWM 构造 ===")
try:
    pwm = PWM(0, 50, duty=0, enable=True)
    print("PWM(通道, 频率, duty=0, enable=True) 成功")
    pwm.deinit()
except Exception as e:
    print("方式1失败:", e)
try:
    pwm = PWM(channel=0, freq=50, duty=0, enable=True)
    print("PWM(channel=0, freq=50, duty=0, enable=True) 成功")
    pwm.deinit()
except Exception as e:
    print("方式2失败:", e)

print("\n=== 初始化摄像头 ===")
sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=320, height=240)
sensor.set_pixformat(Sensor.RGB565)
sensor.run()
img = sensor.snapshot()
print("摄像头 OK")

print("\n=== 测试 find_rects ===")
rects = img.find_rects(threshold=10000)
print("返回类型:", type(rects), "长度:", len(rects))
if rects:
    r = rects[0]
    print("矩形属性:", r.x(), r.y(), r.w(), r.h())

print("\n=== 测试 find_circles ===")
circles = img.find_circles(threshold=2000)
print("返回类型:", type(circles), "长度:", len(circles))
if circles:
    c = circles[0]
    print("圆形属性:", c.x(), c.y(), c.r(), c.magnitude())

print("\n=== 测试 get_regression ===")
img_bin = img.binary([(200, 255)])
line = img_bin.get_regression(thresholds=[(255,255)], robust=True)
if line:
    print("回归线 rho={}, theta={}, mag={}".format(line.rho(), line.theta(), line.magnitude()))
else:
    print("未检测到线（请确保场景中有白色线条）")

print("\n=== 测试 find_template ===")
if hasattr(img, 'find_template'):
    print("find_template 存在")
    # 尝试用当前图像的一部分作为模板
    template = img.copy(roi=(10,10,30,30))
    res = img.find_template(template, threshold=0.5)
    print("模板匹配结果:", res)
else:
    print("find_template 不存在")