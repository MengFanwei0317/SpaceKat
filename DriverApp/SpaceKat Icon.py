import pystray
import PIL.Image
import subprocess
##################################################
### pystray用于创建系统托盘图标                  ###
### PIL.Image用于处理图像                       ###
### subprocess用于启动和管理子进程               ###
##################################################
global SpaceKatDriver
bStarted = False

image = PIL.Image.open("icon.png")
strLogFile = "SpaceKat.log"
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

def miExit(icon, item):
    if bStarted:
        print ("SpaceKat Driver is running, please stop the driver first.")
    else:
        print ("Exit")
        logFile.close()
        icon.stop()

icon = pystray.Icon("SpaceKat", image, menu=pystray.Menu(
    pystray.MenuItem("Start Driver", miStartDriver),
    pystray.MenuItem("Stop Driver", miStopDriver),
    pystray.MenuItem("SpaceKat Settings", miSetting),
    pystray.MenuItem("Exit", miExit)
))

icon.run()
