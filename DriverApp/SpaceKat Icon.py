import pystray
import PIL.Image
import subprocess
import os, sys
##################################################
### pystray用于创建系统托盘图标                  ###
### PIL.Image用于处理图像                       ###
### subprocess用于启动和管理子进程               ###
##################################################
global SpaceKatDriver
bStarted = False
print(os.path.split(os.path.realpath(sys.argv[0]))[0])
ImageFile = os.path.split(os.path.realpath(sys.argv[0]))[0]+"/icon.png"
image = PIL.Image.open(ImageFile)
strLogFile = os.path.split(os.path.realpath(sys.argv[0]))[0]+"/SpaceKat.log"
logFile = open(strLogFile, "w", encoding="utf-8")

#当系统托盘图标被点击时会调用此函数，但目前尚未使用
def on_clicked(icon, item):
    print ("Hello")
    
def miStartDriver(icon, item):
    global SpaceKatDriver
    global bStarted
    if not bStarted:
        SpaceKatDriver = subprocess.Popen('python "SpaceKat Driver.py"', stdout=logFile)   #启动一个新的子进程运行SpaceKat Driver.py脚本，并将输出重定向到日志文件
        bStarted = True
        print ("Start Driver")
    else:
        print ("The driver has been started.")

def miStopDriver(icon, item):
    global SpaceKatDriver
    global bStarted
    if bStarted:
        SpaceKatDriver.terminate()
        print ("Stop Driver")
        bStarted = False
    else:
        print ("The driver has been stopped.")

def miSetting(icon, item):
    subprocess.Popen(['python', 'SpaceKatSetting.py'])
    print ("Setting Window closed.")


# def miSetting(icon, item):
#     global SpaceKatDriver
#     global bStarted
#     subprocess.Popen(['python', 'SpaceKatSetting.py'])
#     if not bStarted:
#         SpaceKatDriver = subprocess.Popen('python "SpaceKat Driver.py"', stdout=logFile)   #启动一个新的子进程运行SpaceKat Driver.py脚本，并将输出重定向到日志文件
#         bStarted = True
#         print ("Config Saved , start Driver.")
#     else:
#         miStopDriver(icon, item)
#         SpaceKatDriver = subprocess.Popen('python "SpaceKat Driver.py"', stdout=logFile)   #启动一个新的子进程运行SpaceKat Driver.py脚本，并将输出重定向到日志文件
#         bStarted = True
#         print ("Config Saved , restart Driver.")

def miExit(icon, item):
    if bStarted:
        print ("SpaceKat Driver is running, stop the driver now.")
        miStopDriver(icon, item)
    print ("Exit")
    logFile.close()
    icon.stop()

icon = pystray.Icon("SpaceKat", image, menu=pystray.Menu(
    pystray.MenuItem("Start Driver", miStartDriver),
    pystray.MenuItem("Stop Driver", miStopDriver),
    pystray.MenuItem("SpaceKat Settings", miSetting),
    pystray.MenuItem("Exit", miExit)
))

# 创建一个空的item
item = None

# 在启动系统托盘之前运行一次miSetting函数
miSetting(icon, item)

icon.run()