#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <wire.h>
#include <TinyUSB_Mouse_and_Keyboard.h>
#include "keycode.h"
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "LittleFS.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define DELAY_INTERVAL_LIMIT 60
#define DELAY_INTERVAL 10
#define SCREEN_WIDTH 128 // OLED 寬度像素
#define SCREEN_HEIGHT 32 // OLED 高度像素
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define ENCODER_L 26
#define ENCODER_R 27

char MenuArray[8][32] = {"Mode", "Fusion360", "JianYing", "Sys Mouse", "SolidWorks\n(TBD)", "Blender\n(TBD)", "PR(TBD)", "Idle"};
int ModeArray[8] = {0, 1, 2, 3, 4, 5, 6, 7};
int SKMode = 1;
int MaxMode = 7;
int tmpSKMode = SKMode;
int ledOn = 1;
int TestParam = 10258;



int LEDPIN = 14;
int NUMPIXELS = 13;
int DelayNum = 0;
int DelayTime = 0;
int DelayNumLimit = DELAY_INTERVAL_LIMIT / DELAY_INTERVAL;
int LEDIndex = 0;
int FlowDirection = -1;
int LEDs[4][5] = {
  {0,1,2,3,4},
  {5,6,7,-1, -1},
  {8,9,-1,-1,-1},
  {10,11,12,-1,-1}
};
int MaxColor = 150;
int ColorDiff = MaxColor / NUMPIXELS;
int ColorBase = 0;
int ColorState = 10;
int ColorStep = MaxColor / ColorState;
int PrevEncoderValue1, EncoderValue1 = 0;

int MenuLevel = 0;
int MenuIndex = 0;
int MenuOperation = 0;    // 0 - Volume Tuning;

byte Cols[] = {4,5,6,7,8};    // Outputs
byte Rows[] = {9,10,11,12};   // Inputs
int CurrentLayer = 1;
char Keys[4][5] = {
  {'R', ' ', ' ', ' ', ' '},
  {'1', '2', '3', '4', '5'},
  {'I', 'C', 'P', ' ', ' '},
  {'-', 'S', ' ', ' ', ' '}
};

int KeyMaps[10][4][5] = {
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KEY_ESC, KP_CTRL+'c', KP_CTRL+'v', KP_CTRL+'z', KP_CTRL+'y'},
    {'m', 'e', SM_MACRO0, KC_NO, KC_NO},
    {KP_CTRL, KEY_LEFT_SHIFT, KC_NO, KC_NO, KC_NO}
  },
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KEY_ESC, KP_CTRL+'c', KP_CTRL+'v', KP_CTRL+'z', KP_CTRL+'y'},
    {'m', 'e', SM_MACRO0, KC_NO, KC_NO},
    {KP_CTRL, KEY_LEFT_SHIFT, KC_NO, KC_NO, KC_NO}
  },
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KEY_ESC, KP_CTRL+'c', KP_CTRL+'v', KP_CTRL+'z', KP_CTRL+KP_SHIFT+'z'},
    {KP_CTRL+'b', 'q', 'w', KC_NO, KC_NO},
    {KP_CTRL, KEY_LEFT_SHIFT, KC_NO, KC_NO, KC_NO}
  },
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO}
  },
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO}
  },
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO}
  },
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO}
  },
  {
    {EC_Click1, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO}
  },
  {
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO}
  },
  {
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO},
    {KC_NO, KC_NO, KC_NO, KC_NO, KC_NO}
  }
};
int longPressDelay = 250/DELAY_INTERVAL;           //customizable keyboard values
int spamSpeed = 15/DELAY_INTERVAL;
int keyDown[4][5];
bool keyLong[4][5];
// Encoder REncoder1(ENCODER_L, ENCODER_R);
int L_LastState;
int L_State;
int R_State;
int LastMillis, CurMillis;
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 3;
int MenuIdleTimer;
int JYYawTimer;

volatile int Encounter = 0;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Tlv493d mag1 = Tlv493d();
Tlv493d mag2 = Tlv493d();
SimpleKalmanFilter xFilter1(0.3, 0.3, 0.99), yFilter1(0.3, 0.3, 0.99), zFilter1(0.3, 0.3, 0.99);
SimpleKalmanFilter xFilter2(0.3, 0.3, 0.99), yFilter2(0.3, 0.3, 0.99), zFilter2(0.3, 0.3, 0.99);
MPU6050 mpu1;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 15  // use pin 2 on Arduino Uno & most boards

float xOffset1 = 0, yOffset1 = 0, zOffset1 = 0;
float xCurrent1 = 0, yCurrent1 = 0, zCurrent1 = 0;
float xOffset2 = 0, yOffset2 = 0, zOffset2 = 0;
float xCurrent2 = 0, yCurrent2 = 0, zCurrent2 = 0;
int calSamples = 50;//偏移校准采样次数
int sensivity = 8;
float magRange = 3.0;
float outRange = 127.0;      // Max allowed in HID report
float xyThreshold = 0.3; // Center threshold
float panLRThreshold = 0.35;
float panFBThreshold = 0.25;
float panUDThreshold = 0.4;
float pitchThreshold = 1.5;
float rollThreshold = 1.5;
float yawThreshold = 1.5;
float inRange = magRange * sensivity;
float zThreshold = 0.3;
float yawState = 10000;

float xPos1[] = {0, 0, 0}, yPos1[] = {0, 0, 0}, zPos1[] = {0, 0, 0};
float xPos2[] = {0, 0, 0}, yPos2[] = {0, 0, 0}, zPos2[] = {0, 0, 0};
int PosIndex=0;
int PrevMS = 0;
int Counter = 0;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
  Mouse.begin();//启动鼠标模拟
  Keyboard.begin();//启动键盘模拟

  Serial.begin(115200);//启动串口，设置波特率115200
  while (!Serial);//等待串口初始化完成
  Serial.println("Serial Ready.");

  Serial.println("Read Config");
  LittleFS.begin();//初始化于ESP8266和ESP32等芯片的文件系统LittleFs
  char buff[128];//初始化用于存储配置信息的字符数组
  int cnt = 1;//初始化计数器cnt并赋值为1
  if (LittleFS.exists("SpaceKat.cfg"))//判断是否存在SpaceKat.cfg配置文件
  {
    File f = LittleFS.open("SpaceKat.cfg", "r");//以只读模式打开SpaceKat.cfg并返回对象f
    if (f) {
      bzero(buff, 128);//将buff全部清零
      if (f.read((uint8_t *)buff, 128)) {//从f中读取128位数据存储到buff中
        sscanf(buff, "%d, %d, %d", &SKMode, &ledOn, &TestParam);//从buff中读取三个整数存储在SKMode、ledon、TestParam中
        Serial.printf("Load configuration. SKMode:%d, LED On:%d, Test Param:%d", SKMode, ledOn, TestParam);
        CurrentLayer = SKMode;//将配置中的Mode传递给CurrentLayer
    }
    f.close();
    }
  }
  else
  {
    Serial.println("Configure file not exist.");
  }
  

  pixels.begin();//初始化NeoPixel像素控制器
  pixels.setPixelColor(10, 0, 0, 0);//将编号为10、11、12的像素点颜色设置为黑色
  pixels.setPixelColor(11, 0, 0, 0);
  pixels.setPixelColor(12, 0, 0, 0);
  pixels.show();

  for (int i=0; i<=3; i++)
  {
    pinMode(Rows[i], OUTPUT);//将矩阵键盘中的行Rows设置为输出
    digitalWrite(Rows[i], HIGH);//将Rows置高
  }
  for (int i=0; i<=4; i++)
  {
    pinMode(Cols[i], INPUT_PULLUP);//将矩阵键盘中的列设置为输入，并使用内部上拉电阻
  }

  pinMode(ENCODER_L, INPUT_PULLUP);//将编码器的左右引脚设置为输入，并使用内部上拉电阻
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), read_encoder, CHANGE);//将编码器左右引脚状态和read_encoder函数关联，引脚状态发生改变时触发中断
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), read_encoder, CHANGE);




  Serial.println("Wire Begin");//串口发送I2C总线初始化消息

  /*
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  */

  Wire.setSDA(0u);//初始化第一个I2C总线，数据线连接0脚，时钟线连接1脚
  Wire.setSCL(1u);
  Wire.begin();
  // Wire.setClock(400000);

  Wire1.setSDA(2);//初始化第二个I2C总线，数据线连接2脚，时钟线连接3脚
  Wire1.setSCL(3);
  Wire1.begin();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 一般1306 OLED的位址都是0x3C
    Serial.println(F("SSD1306 allocation failed"));
    // for(;;); // Don't proceed, loop forever
  }
  else{
    display.ssd1306_command(SSD1306_SEGREMAP);//翻转显示，将显示内容上下翻转
    display.ssd1306_command(SSD1306_COMSCANINC);//正向行扫描（从上至下）
    display.display();//更新显示器
    delay(1000);
    display.clearDisplay();//清除显示内容
    display.setTextSize(2);//设置文字大小两倍            
    display.setTextColor(1); //设置文字颜色白色
    display.setCursor(0,0);//设置光标位置左上角
    display.print("Welcome to SpaceKat");//显示欢迎字符串
    display.display(); //更新
    delay(300); 
    display.clearDisplay();//清除
    display.setTextSize(1.5);//字号1.5
    display.setCursor(0,0);//光标左上角
    display.println("Initializing...");
    display.println("Please Do NOT move.");
    display.display();
    delay(100);
  }


  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again
  Serial.println("Setup Started...");
  MagBegin();//初始化Tlv493d
  delay(1);
  MPUBegin();//初始化MPU6050

  L_LastState = digitalRead(ENCODER_L);//读取编码器左引脚状态
  LastMillis = millis();//获取当前时间的毫秒数，存储到LastMillis中
  MenuIdleTimer = millis();//初始化菜单空闲计时器
  JYYawTimer = millis();//初始化JYawTimer，用于检测JY航向时间

  display.clearDisplay();
  display.setTextSize(2);            
  display.setTextColor(1); 
  display.setCursor(0,0);
  display.print("Ready!");
  display.display(); 
  delay(100); 
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Mode: ");
  display.println(MenuArray[SKMode]);
  display.display();

}

void MagBegin()//初始化Tlv493d
{


  // mag sensor init
  Serial.println("Mag1 Begin ");
  mag1.begin(Wire1);//Tlv493d磁力计1使用第二条I2C总线
  mag1.setAccessMode(mag1.LOWPOWERMODE);//设置Tlv493dmag1访问模式为低功耗模式
  mag1.disableTemp();//禁用温度传感功能

  // crude offset calibration on first boot
  //进行偏移校准
  Serial.println("Cal Offset...");
  for (int i = 1; i <= calSamples; i++)
  {

    //delay(mag1.getMeasurementDelay());
    delay(30);
    mag1.updateData();//更新Tlv493dmag1的数据

    xOffset1 += mag1.getX();//累加三轴偏移量
    yOffset1 += mag1.getY();
    zOffset1 += mag1.getZ();

    Serial.print(".");
  }

  xOffset1 = xOffset1 / calSamples;//计算平均偏移量
  yOffset1 = yOffset1 / calSamples;
  zOffset1 = zOffset1 / calSamples;

  Serial.println();//串口输出xyz偏移量
  Serial.println(xOffset1);
  Serial.println(yOffset1);
  Serial.println(zOffset1);

  Serial.println("Mag2 Begin ");
  mag2.begin(Wire);//Tlv493d磁力计2使用第一条I2C总线
  mag2.setAccessMode(mag2.LOWPOWERMODE);//低功耗
  mag2.disableTemp();//关闭温度传感

  // crude offset calibration on first boot
  Serial.println("Cal Offset...");
  for (int i = 1; i <= calSamples; i++)
  {

    //delay(mag2.getMeasurementDelay());
    delay(30);
    mag2.updateData();

    xOffset2 += mag2.getX();
    yOffset2 += mag2.getY();
    zOffset2 += mag2.getZ();

    Serial.print(".");
  }

  xOffset2 = xOffset2 / calSamples;
  yOffset2 = yOffset2 / calSamples;
  zOffset2 = zOffset2 / calSamples;

  Serial.println();
  Serial.println(xOffset2);
  Serial.println(yOffset2);
  Serial.println(zOffset2);
}

void MPUBegin()//初始化MPU6050
{


    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    // Serial.begin(9600);
    // while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));//F（）是一个宏，将字符串存储在Flash存储器上而不是RAM中
    mpu1.initialize();//初始化MPU6050
    // pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu1.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));//根据mpu1.testConnection()返回值输出测试结果

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));//提示用户发送任意字符串以进行DMP演示
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu1.dmpInitialize();//初始化dmp，如果成功返回0

    mpu1.setXGyroOffset(61);//配置xyz轴陀螺仪偏移量
    mpu1.setYGyroOffset(46);
    mpu1.setZGyroOffset(117);
    mpu1.setXAccelOffset(1740); // 配置xyz轴加速度计偏移量
    mpu1.setYAccelOffset(-935); 
    mpu1.setZAccelOffset(934); 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {//如果dmp启动成功则继续进行校准
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu1.CalibrateAccel(6);//校准加速度计
        mpu1.CalibrateGyro(6);//校准陀螺仪
        mpu1.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu1.setDMPEnabled(true);//打开dmp

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));//表示在INTERRUPT_PIN引脚上启动外部中断检测
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        //mpuIntStatus = mpu1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;//将dmp标志位配置为真

        // get expected DMP packet size for later comparison
        // packetSize = mpu1.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  int ColorIndex;

  for (i=0; i<=3; i++)
  {
    digitalWrite(Rows[i], LOW);
    delayMicroseconds(5);

    for (int j=0; j<=4; j++)
    {
      if (digitalRead(Cols[j]) == LOW)
      {
        keyPressed(i,j); 
      }
      else if (keyDown[i][j] != 0)
      {
        resetKey(i,j);
      }
    }
    digitalWrite(Rows[i], HIGH);
    delayMicroseconds(500);
  }

  if (DelayNum % DelayNumLimit == 0)
  {
    ColorBase ++;
    ColorBase = ColorBase % (ColorState * 3);



    int tmpColorR, tmpColorG, tmpColorB;
    for (i=0; i<=NUMPIXELS-4 ; i++)
    {
      ColorIndex = ColorBase + i;
      ColorIndex = ColorIndex % (ColorState * 3);
      if (ColorIndex < ColorState)
      {
        pixels.setPixelColor(NUMPIXELS-4-i, MaxColor-ColorIndex*ColorStep, ColorIndex*ColorStep, 0);
      }
      if ((ColorIndex >= ColorState) and (ColorIndex < ColorState*2))
      {
        pixels.setPixelColor(NUMPIXELS-4-i, 0, MaxColor-(ColorIndex-ColorState)*ColorStep, (ColorIndex-ColorState)*ColorStep);
      }
      if ((ColorIndex >= ColorState*2) && (ColorIndex < ColorState*3))
      {
        pixels.setPixelColor(NUMPIXELS-4-i, (ColorIndex-ColorState*2)*ColorStep, 0, MaxColor-(ColorIndex-ColorState*2)*ColorStep);
      }
      // pixels.show();

    }
    for (i=0; i<=2; i++)
    {
      ColorIndex = ColorBase + i;
      ColorIndex = ColorIndex % (ColorState * 3);

      if (ColorIndex < ColorState)
      {
        pixels.setPixelColor(NUMPIXELS-1-i, MaxColor-ColorIndex*ColorStep, ColorIndex*ColorStep, 0);
      }
      if ((ColorIndex >= ColorState) and (ColorIndex < ColorState*2))
      {
        pixels.setPixelColor(NUMPIXELS-1-i, 0, MaxColor-(ColorIndex-ColorState)*ColorStep, (ColorIndex-ColorState)*ColorStep);
      }
      if ((ColorIndex >= ColorState*2) && (ColorIndex < ColorState*3))
      {
        pixels.setPixelColor(NUMPIXELS-1-i, (ColorIndex-ColorState*2)*ColorStep, 0, MaxColor-(ColorIndex-ColorState*2)*ColorStep);
      }
      pixels.show();
    }
    /*
    if (FlowDirection == 1)
    {
      if (LEDIndex <= 9)
        pixels.setPixelColor(LEDIndex, 0, 250-15*LEDIndex, 15*LEDIndex);
    }
    else
    {
      pixels.setPixelColor(LEDIndex, 15*LEDIndex, 0, 250-15*LEDIndex); 
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
    */
    // EncoderValue1 = REncoder1.read();
    // Serial.println(EncoderValue1);

    if (LEDIndex >= 10)
    {
      FlowDirection = -1;
    }
    if (LEDIndex <= -1)
    {
      FlowDirection = 1;
    }
    LEDIndex = LEDIndex + FlowDirection;
  }



  static int lastCounter = 0;
  if (MenuOperation == 0)
  {
    if(Encounter > lastCounter){
      Serial.println("Volume,INC");
      lastCounter = Encounter;
    }
    if (Encounter < lastCounter)
    {
      Serial.println("Volume,DEC");
      lastCounter = Encounter;
    }
  }
  else if (MenuOperation == 1)
  {
    if(Encounter > lastCounter){    // Right
      lastCounter = Encounter;
      if (tmpSKMode < MaxMode)
      {
        tmpSKMode += 1;
        OLEDDisplay(MenuArray[tmpSKMode], 2);
      }
    }
    if (Encounter < lastCounter)    // Left
    {
      lastCounter = Encounter;
      if (tmpSKMode >= 2)
      {
        tmpSKMode -= 1;
        OLEDDisplay(MenuArray[tmpSKMode], 2);
      }
    }
  }
  

  
  delay(DELAY_INTERVAL); // Delay for a period of time (in milliseconds).
  DelayNum++;


  if (millis()-PrevMS >= 1000)
  {
    Serial.print("Counter = ");
    Serial.println(Counter);
    PrevMS = millis();
    Counter = 0;
  }
  Counter++;

  mag1.updateData();
  mag2.updateData();
  
  // update the filters
  xCurrent1 = xFilter1.updateEstimate(mag1.getX() - xOffset1);
  yCurrent1 = yFilter1.updateEstimate(mag1.getY() - yOffset1);
  zCurrent1 = zFilter1.updateEstimate(mag1.getZ() - zOffset1);

  
  xCurrent2 = xFilter2.updateEstimate(mag2.getX() - xOffset2);
  yCurrent2 = yFilter2.updateEstimate(mag2.getY() - yOffset2);
  zCurrent2 = zFilter2.updateEstimate(mag2.getZ() - zOffset2);


  // check the center threshold
  if (abs(xCurrent1) > xyThreshold || abs(yCurrent1) > xyThreshold)
  {

  }


    // if programming failed, don't try to do anything
    float xData = (xCurrent1 + xCurrent2) / 2.0;
    float yData = (yCurrent1 + yCurrent2) / 2.0;
    float zData = (zCurrent1 + zCurrent2) / 2.0;
    float xMove, yMove, zMove, xRotate, yRotate, zRotate;
    int xMouse, yMouse, zMouse, xRMouse, yRouse, zRMouse;
    if (SKMode == 2)
    {
      if (abs(xData) > panFBThreshold)
      {
        xMove = xData > 0?xData-panFBThreshold:xData+panFBThreshold;
        xMouse = round(xMove / magRange * outRange);
      }
      else
        xMouse = 0;
      if (abs(yData) > panLRThreshold)
      {
        yMove = yData > 0?yData-panLRThreshold:yData+panLRThreshold;
        yMouse = round(yMove / magRange * outRange);
      }
      else
        yMouse = 0;

      if (abs(zData) > panUDThreshold)
      {
        zMove = zData > 0?zData-panUDThreshold:zData+panUDThreshold;
        zMouse = round(zMove / magRange * outRange);
      }
      else
        zMouse = 0;

      if (dmpReady)
      {
        if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer)) 
        { // Get the Latest packet 
            // display Euler angles in degrees
            mpu1.dmpGetQuaternion(&q, fifoBuffer);
            mpu1.dmpGetGravity(&gravity, &q);
            mpu1.dmpGetYawPitchRoll(ypr, &q, &gravity);
            float yawCurrent = ypr[0] * 180 / M_PI;
            if (yawState >= 9000)
              yawState = yawCurrent;
            // char tmpBuffer[127];
            // sprintf (tmpBuffer, "yawCurrent: %f, yCurrent1: %f, yCurrrent2: %f, yawState: %f", yawCurrent, yCurrent1, yCurrent2, yawState);
            // Serial.println(tmpBuffer);
            zRMouse = 0;
            if (abs(yCurrent1-yCurrent2) < 0.3)
              yawState = yawCurrent;
            else
            {
              float Diff = yawCurrent - yawState;
              if (abs(Diff) > yawThreshold)
              {
                zRotate = Diff > 0?Diff-yawThreshold:Diff+yawThreshold;
                zRMouse = round(zRotate / 10 * outRange);
              }
              else
                zRMouse = 0;
              // sprintf(tmpBuffer, "Diff: %f, zRotate: %f, zRMouse: %f", Diff, zRotate, zRMouse);
              // Serial.println(tmpBuffer);
            }
        }
        else zRMouse = 0;
      }
      else
        zRMouse = 0;

      // Serial.print("zRMouse: ");
      // Serial.println(zRMouse);
      if (zRMouse > 50)
      {
        Keyboard.press(KEY_LEFT_SHIFT);
        Keyboard.write(KEY_RIGHT_ARROW);
        Keyboard.release(KEY_LEFT_SHIFT);
      }
      else if (zRMouse > 25)
      {
        Keyboard.write(KEY_RIGHT_ARROW);
      }
      else if (zRMouse > 0.1)
      {
        int curTimer = millis();
        if (curTimer - JYYawTimer > 300)
        {
          Keyboard.write(KEY_RIGHT_ARROW);
          JYYawTimer = curTimer;
        }
      }
      else if (zRMouse < -50)
      {
        Keyboard.press(KEY_LEFT_SHIFT);
        Keyboard.write(KEY_LEFT_ARROW);
        Keyboard.release(KEY_LEFT_SHIFT);
      }
      else if (zRMouse < -25)
      {
        Keyboard.write(KEY_LEFT_ARROW);
      }
      else if (zRMouse < -0.1)
      {
        int curTimer = millis();
        if (curTimer - JYYawTimer > 300)
        {
          Keyboard.write(KEY_LEFT_ARROW);
          JYYawTimer = curTimer;
        }
      }

      if (yMouse > 1)
      {
        Keyboard.press(KEY_LEFT_ALT);
        Mouse.move(0, 0, 1);
        Keyboard.release(KEY_LEFT_ALT);
      }
      else if (yMouse < -1)
      {
        Keyboard.press(KEY_LEFT_ALT);
        Mouse.move(0, 0, -1);
        Keyboard.release(KEY_LEFT_ALT);
      }

      if (zMouse > 1)
      {
        Keyboard.press(KEY_LEFT_CTRL);
        Mouse.move(0, 0, -1);
        Keyboard.release(KEY_LEFT_CTRL);
      }
      else if (zMouse < -1)
      {
        Keyboard.press(KEY_LEFT_CTRL);
        Mouse.move(0, 0, 1);
        Keyboard.release(KEY_LEFT_CTRL);
      }


      
      return;

    }
    if (SKMode == 3)
    {
      if (abs(xData) > panLRThreshold)
      {
        xMove = xData > 0?xData-panLRThreshold:xData+panLRThreshold;
        xMouse = round(xMove / magRange * outRange);
      }
      else
        xMouse = 0;
      if (abs(yData) > panFBThreshold)
      {
        yMove = yData > 0?yData-panLRThreshold:yData+panLRThreshold;
        yMouse = round(yMove / magRange * outRange);
      }
      else
        yMouse = 0;

      if (xMouse != 0 || yMouse != 0)  
        Mouse.move(yMouse, -xMouse, 0);
      return;

    }
    if (SKMode == 7)
      return;
    if (!dmpReady)
    {
      Serial.print("Data,");
      Serial.print(yCurrent1);
      Serial.print(",");
      Serial.print(xCurrent1);
      Serial.print(",");
      Serial.print(zCurrent1);
      Serial.print(",");
      Serial.print(yCurrent2);
      Serial.print(",");
      Serial.print(xCurrent2);
      Serial.print(",");
      Serial.print(zCurrent2);
      Serial.println(",0,0,0,");
      return;
    }
    // read a packet from FIFO
    if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu1.dmpGetQuaternion(&q, fifoBuffer);
            mpu1.dmpGetGravity(&gravity, &q);
            mpu1.dmpGetYawPitchRoll(ypr, &q, &gravity);
          //  Serial.print("ypr\t");
          //  Serial.print(ypr[0] * 180/M_PI);
          //  Serial.print("\t");
          //  Serial.print(ypr[1] * 180/M_PI);
          //  Serial.print("\t");
          //  Serial.println(ypr[2] * 180/M_PI);
            Serial.print("Data,");
            Serial.print(yCurrent1);
            Serial.print(",");
            Serial.print(xCurrent1);
            Serial.print(",");
            Serial.print(zCurrent1);
            Serial.print(",");
            Serial.print(yCurrent2);
            Serial.print(",");
            Serial.print(xCurrent2);
            Serial.print(",");
            Serial.print(zCurrent2);
            Serial.print(",");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print(",");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print(",");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print(",");
            Serial.println();
            
        #endif


    }
 

}


void keyPressed(int row, int col){
/*
  Serial.print("Output: "); 
  Serial.print(row);
  Serial.print(" Input: ");
  Serial.print(col);
  Serial.print(" ");
  Serial.println(Keys[row][col]);
*/
  if(keyDown[row][col]==0){         //if the function is called for the first time for this key
    // Keyboard.write(Keys[row][col]);
    keyPressAction(KeyMaps[CurrentLayer][row][col]);
  }
  else if(keyLong[row][col] && keyDown[row][col] > spamSpeed){ //if the key has been held long enough to warrant another keystroke set
    // Keyboard.write(Keys[row][col]);
    keyPressAction(KeyMaps[CurrentLayer][row][col]);
    keyDown[row][col] = 1;
  }
  else if(keyDown[row][col] > longPressDelay){ //if the key has been held for longer that longPressDelay, it switches into spam mode
    keyLong[row][col] = true;
  }

  keyDown[row][col]++;
/*
  Serial.print(keyLong[row][col]);
  Serial.print(", ");
  Serial.println(keyDown[row][col]);
*/
}

void resetKey(int row, int col){ //resetting the variables after key is released
  keyDown[row][col] = 0;
  keyLong[row][col] = false;
  keyResetAction(KeyMaps[CurrentLayer][row][col]);
}

void keyPressAction(int KeyCode)
{
  if (KeyCode >= KC_BASE && KeyCode <= KC_MAX)    // Direct Input Key
  {
    switch (KeyCode)
    {
      case KEY_LEFT_SHIFT:
        Keyboard.press(KEY_LEFT_SHIFT);
        break;
      case KEY_LEFT_CTRL:
        Keyboard.press(KEY_LEFT_CTRL);
        break;
      case KEY_LEFT_ALT:
        Keyboard.press(KEY_LEFT_ALT);
        break;
      default:
        Keyboard.write(KeyCode);
        break;
    }
  }

  else if (KeyCode >= KP_BASE && KeyCode <= KP_MAX)   // Crtl | Alt | Shift | FN + Input Key
  {
    if ((KeyCode & KP_CTRL) != 0)      // Ctrl Pressed
    { 
      Serial.println("Control Key");
      Keyboard.press(KEY_LEFT_CTRL);
    }
    if ((KeyCode & KP_ALT) != 0)
      Keyboard.press(KEY_LEFT_ALT);
    if ((KeyCode & KP_SHIFT) != 0)
      Keyboard.press(KEY_LEFT_SHIFT);

    int KC_Code = KeyCode & 0x00FF;
    Serial.println(KC_Code, HEX);
    Keyboard.press(KC_Code);
    Keyboard.release(KC_Code);
  }
  else if (KeyCode >= SM_BASE && KeyCode <= EC_Base)
  {
    if (KeyCode == SM_MACRO0)
    {
      Serial.println("SM_MACRO,FIND");
    }
    
  }
  else if (KeyCode >= EC_Base)
  {
    if (KeyCode == EC_Click1)
    {
      Serial.println(MenuLevel);
      if (MenuLevel == 0)
      {
        OLEDDisplay(MenuArray[SKMode], 2);
        MenuLevel += 1;
        MenuOperation = 1;
        tmpSKMode = SKMode;
      }
      else if (MenuLevel == 1 && MenuOperation == 1)
      {
        MenuLevel = 0;
        MenuOperation = 0;
        SKMode = tmpSKMode;
        CurrentLayer = SKMode;
        char tmpBuffer[127];
        sprintf(tmpBuffer, "Mode: \n%s", MenuArray[SKMode]);
        OLEDDisplay(tmpBuffer, 2);
        SaveConfig();
      }
    }
  }
  

}

void SaveConfig()
{
  char buff[128];
  File f = LittleFS.open("SpaceKat.cfg", "w");
  bzero(buff, 128);
  sprintf(buff, "%d, %d, %d", SKMode, ledOn, TestParam);
  if (f) {
    Serial.print("Save configuration:");
    Serial.println(buff);
    f.write(buff, strlen(buff));
    f.close();
  }
  else
  {
    Serial.println("Cannot save configuration.");
  }
}
void OLEDDisplay(char * str, float fontSize)
{
  display.clearDisplay();
  display.setTextSize(fontSize);            
  display.setCursor(0,0);
  display.print(str);
  display.display();
  // delay(1);
}

void keyResetAction(int KeyCode)
{
  if (KeyCode >= KC_BASE && KeyCode <= KC_MAX)    // Direct Input Key
  {
    switch (KeyCode)
    {
      case KEY_LEFT_SHIFT:
        Keyboard.release(KEY_LEFT_SHIFT);
        break;
      case KEY_LEFT_CTRL:
        Keyboard.release(KEY_LEFT_CTRL);
        break;
      case KEY_LEFT_ALT:
        Keyboard.release(KEY_LEFT_ALT);
        break;
    }
  }
  
  if (KeyCode >= KP_BASE && KeyCode <= KP_MAX)   // Crtl | Alt | Shift | FN + Input Key
  {
    if ((KeyCode & KP_CTRL) != 0)      // Ctrl Pressed
      Keyboard.release(KEY_LEFT_CTRL);
    if ((KeyCode & KP_ALT) != 0)
      Keyboard.release(KEY_LEFT_ALT);
    if ((KeyCode & KP_SHIFT) != 0)
      Keyboard.release(KEY_LEFT_SHIFT);


  }
}

void read_encoder()
{
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENCODER_L)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENCODER_R)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = -1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    Encounter = Encounter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = 1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    Encounter = Encounter + changevalue;              // Update counter
    encval = 0;
  }
}
