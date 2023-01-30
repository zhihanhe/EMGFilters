/*
 * Copyright 2017, OYMotion Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"

// https://arduinojson.org/
// 解析json
#include "ArduinoJson-v6.20.0.h"

#define GMEDEBUG      0

// Analog input pins
int SensorInputPins[] = {A0,A1,A2};

#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))

// Define the `CALIBRATE` macro as 1 to calibrate the baseline value
// of input sEMG signals.
//
// After wiring the sEMG sensors to the Arduino board, wear the
// sEMG sensors. Relax your muscles for a few seconds, you
// will be able to see a series of squared sEMG signals values get printed on
// your serial terminal. Choose the maximal one as the baseline by setting
// the `baseline` variable.
//
// After calibriting, change the `CALIBRATE` macro to 0, and rebuild this
// project. The `envelope`, which is the squared sEMG signal data, will be
// printed to the serial line. The developer can plot it using the Arduino
// SerialPlotter.
//
// Note:
//      After calibration, Any squared value of sEMG sigal below the
//      baseline will be treated as zero.
//
//      It is recommended that you do calibration every time you wear
//      the sEMG sensor.
// 将 `CALIBRATE` 宏定义为 1 以校准基线值
// 输入 sEMG 信号。
//
// 将 sEMG 传感器连接到 Arduino 板上后，佩戴
// sEMG 传感器。 放松你的肌肉几秒钟，你
// 将能够看到打印的一系列平方 sEMG 信号值
// 你的串行终端。 通过设置选择最大的一个作为基线
// `baseline` 变量。
//
// 校准后，将 `CALIBRATE` 宏更改为 0，并重建它
// 项目。 “包络”，即平方 sEMG 信号数据，将是
//打印到串行线。 开发人员可以使用 Arduino 绘制它
//串行绘图仪。
//
// 笔记：
// 校准后，sEMG 信号的任何平方值低于
// 基线将被视为零。
//
// 建议每次佩戴都要校准
// sEMG 传感器。
// #define CALIBRATE 1
int currentModel = -1;

int baseline = 200;

EMGFilters myFilter[ARR_SIZE(SensorInputPins)];

// Set the input frequency.
//
// The filters work only with fixed sample frequency of
// `SAMPLE_FREQ_500HZ` or `SAMPLE_FREQ_1000HZ`.
// Inputs at other sample rates will bypass
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;

// Time interval for processing the input signal.
// unsigned long long interval = 1000000ul / sampleRate;
unsigned long long interval = 1ul / sampleRate;

// Set the frequency of power line hum to filter out.
//
// For countries with 60Hz power line, change to "NOTCH_FREQ_60HZ"
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;


void setup() {
    /* add setup code here */
    // myFilter.init(sampleRate, humFreq, true, true, true);
    
    for (int i = 0; i < ARR_SIZE(SensorInputPins); i++) {
      myFilter[i].init(sampleRate, humFreq, true, true, true);
    }

    // open serial
    Serial.begin(115200);

    // Serial.setTimeout(30000);

    // 保证数据读完
    while(Serial.read() >= 0){}
}

void loop() {
  readDate();
  checkDate();
}

// 采集反馈
void readDate() {
  // 测试读取
  if (Serial.available() != 0) {
    //等数据传完
    // delay(100);
    String val = Serial.readString();
    // 太大了就内存溢出了！！！！注意内存
    StaticJsonDocument<256> receive;
    deserializeJson(receive, val);
    //const char* action = receive["action"];
    const int model = receive["model"];
    switch(model) {
      // 采集模式
      case 0:{
        Serial.print("修改成采集模式,userBaseLine:");
        currentModel = model;
        int userBaseLine = receive["data"]["baseline"];
        baseline = userBaseLine;
        Serial.println(userBaseLine);
        break;
      }
      // 调试模式
      case 1: {
        Serial.println("修改成调试模式");
        currentModel = model;
        break;
      }
      default: {
        Serial.print("还未定义的模式:");
        Serial.println(model);
      }
    }
    // 清空串口缓存
    while(Serial.read() >= 0){}
  }
}

// 在指定模式下执行采集动作
void checkDate() {
  if(currentModel == -1) {
    return;
  }

  // Note: `micros()` will overflow and reset every about 70 minutes.
  unsigned long long timeStamp = micros();

  // 太大了就内存溢出了！！！！注意内存
  StaticJsonDocument<256> send;
  send["model"] = currentModel;

  JsonArray dataArray = send.createNestedArray("data");

  for (int i = 0; i < ARR_SIZE(SensorInputPins); i++) {

    int data = analogRead(SensorInputPins[i]);
    int dataAfterFilter = myFilter[i].update(data);
    // Get envelope by squaring the input
    // 这个数据算了，放到前端去算
    // int envelope = sq(dataAfterFilter);
    int absValue = abs(dataAfterFilter);

    switch(currentModel) {
      // 采集模式
      case 0: {
        // Any value below the `baseline` value will be treated as zero
        //if (envelope < baseline) {
        if (absValue < baseline) {
            dataAfterFilter = 0;
            // envelope = 0;
        }
        // filter processing
        StaticJsonDocument<56> jData;
        jData["k"] = SensorInputPins[i];
        jData["v"] = dataAfterFilter;
        dataArray.add(jData);
        break;
      }
      // 调试模式
      case 1: {
        StaticJsonDocument<56> jData;
        jData["k"] = SensorInputPins[i];
        jData["v"] = dataAfterFilter;
        dataArray.add(jData);
        // You may plot the data using Arduino SerialPlotter.
        // Serial.print(envelope);
        break;
      }

    }
  }

  // 发出消息
  serializeJson(send, Serial);
  Serial.println();

  // Usually, you should still have (interval - timeElapsed) to do other work.
  // Otherwise, you would have to lower down the `sampleRate`.
  unsigned long timeElapsed = micros() - timeStamp;
#if GMEDEBUG
  // 太大了就内存溢出了！！！！注意内存
  StaticJsonDocument<96> debug;
  debug["info"] = "cost";
  debug["time"] = timeElapsed;
  serializeJson(debug, Serial);
  Serial.println();

  // Serial.print("Filters cost time: ");
  // Serial.println(timeElapsed);
#else
  // unsigned long delayTime = (interval - timeElapsed) / 1000;
  // Serial.print("delay(ms):");
  // Serial.println(delayTime);
  
#endif
  delay((timeElapsed)/1000+10);
}






// /*
//  * 无锡市思知瑞科技有限公司
//  * 淘宝店铺：大脑实验室
//  * 店铺网址：http://brainlab.taobao.com
//  * 工作日联系电话：0510-66759621
//  */
// #if defined(ARDUINO) && ARDUINO >= 100
// #include "Arduino.h"
// #else
// #include "WProgram.h"
// #endif
// #include "EMGFilters.h"
// #define  SensorInputPin A1       //sensor input pin number
// unsigned long threshold = 300;      // threshold: Relaxed baseline values.(threshold=0:in the calibration process)
// unsigned long EMG_num = 0;      // EMG_num: The number of statistical signals
// EMGFilters myFilter;
// SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ;
// NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;
// void setup() 
// {
//   myFilter.init(sampleRate, humFreq, true, true, true);
//   Serial.begin(115200);
// }
// void loop() 
// {
//   int data = analogRead(SensorInputPin);
//   int dataAfterFilter = myFilter.update(data);  
//   int envelope = sq(dataAfterFilter);   
//   envelope = (envelope > threshold) ? envelope : 0;    
//   if (threshold > 0) 
//   {
//     if (getEMGCount(envelope)) 
//     {  EMG_num++;
//       Serial.print("EMG_num: ");
//       Serial.println(EMG_num);
//     }
//   }
//     /*
//  * 无锡市思知瑞科技有限公司
//  * 淘宝店铺：大脑实验室
//  * 店铺网址：http://brainlab.taobao.com
//  * 工作日联系电话：0510-66759621
//  */
//   else {
//     Serial.println(envelope);
//   }
//   delayMicroseconds(500);
// }
// int getEMGCount(int gforce_envelope)
// {
//   static long integralData = 0;
//   static long integralDataEve = 0;
//   static bool remainFlag = false;
//   static unsigned long timeMillis = 0;
//   static unsigned long timeBeginzero = 0;
//   static long fistNum = 0;
//   static int  TimeStandard = 200;
//   integralDataEve = integralData;
//   integralData += gforce_envelope;  
//   if ((integralDataEve == integralData) && (integralDataEve != 0)) 
//   {
//     timeMillis = millis();
//     if (remainFlag) 
//     {
//       timeBeginzero = timeMillis;
//       remainFlag = false;
//       return 0;
//     }
//       /*
//  * 无锡市思知瑞科技有限公司
//  * 淘宝店铺：大脑实验室
//  * 店铺网址：http://brainlab.taobao.com
//  * 工作日联系电话：0510-66759621
//  */
//     if ((timeMillis - timeBeginzero) > TimeStandard) 
//     {
//       integralDataEve = integralData = 0;
//       return 1;
//     }
//     return 0;
//   }
//   else {
//     remainFlag = true;
//     return 0;
//    }
// }