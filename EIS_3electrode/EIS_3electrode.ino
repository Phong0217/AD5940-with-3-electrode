/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Electrochemical impedance spectroscopy based on example AD5940_Impedance
							This project is optomized for 3-lead electrochemical sensors that typically have 
							an impedance <200ohm. For optimum performance RCAL should be close to 
							impedance of the sensor.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "Impedance.h"

// #include <WiFi.h>
// #include <WebServer.h>


// const char *ssid = "ESP32-Access-Point";
// const char *password = "12345678";

// IPAddress local_ip(192,168,49,15);
// IPAddress gateway(192,168,49,1);
// IPAddress subnet(255,255,255,0);

// WebServer server(80);

unsigned long timeStart = 0;
unsigned long timeNow = 0;

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

float S_Vol, E_Vol;
int countRepeat, StepNumber, RepeatTimes = 0;
BoolFlag logEn = bFALSE;
String inputString = "";
byte moc1, moc2, moc3, moc4, moc5, moc 6, moc 7;

#define FILTER_SIZE 101
float filteredMagnitude[FILTER_SIZE];
float filteredPhase[FILTER_SIZE];
int filterIndex = 0;

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  if (S_Vol == E_Vol)
  {
    timeNow = millis() - timeStart;
    printf("%lu;", timeNow);
  }
  else
  {
    printf("%.2f;", freq);
  }
  /*Process data*/
  float phase;
  for(int i=0;i<DataCount;i++)
  {
    phase = pImp[i].Phase*180/MATH_PI;
    if(phase > 180) phase = phase - 360;
    else if (phase < -180) phase = phase + 360;
    //printf("%f;%f\n", pImp[i].Magnitude, pImp[i].Phase*180/MATH_PI);
    printf("%f;%f\n", pImp[i].Magnitude, phase);
  }
  return 0;
}


// #define MAX_DATA_COUNT 100 // Số lượng dữ liệu tối đa bạn muốn lưu trữ

// float freqArray[MAX_DATA_COUNT]; // Mảng lưu trữ tần số
// float magnitudeArray[MAX_DATA_COUNT]; // Mảng lưu trữ trở kháng
// float phaseArray[MAX_DATA_COUNT]; // Mảng lưu trữ pha

// int dataCount = 0; // Số lượng dữ liệu hiện tại

// int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
// {
//   float freq;

//   fImpPol_Type *pImp = (fImpPol_Type*)pData;
//   AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

//   // Lưu tần số vào mảng
//   freqArray[dataCount] = freq;

//   printf("Freq:%.2f ", freq);
//   /*Process data*/
//   for(int i=0;i<DataCount;i++)
//   {
//     // Lưu trở kháng vào mảng
//     magnitudeArray[dataCount] = pImp[i].Magnitude;

//     // Lưu pha vào mảng
//     phaseArray[dataCount] = pImp[i].Phase * 180 / MATH_PI;

//     // Tăng biến đếm số lượng dữ liệu
//     dataCount++;

//     if(dataCount == DataCount){
//       delay(1000);
//       dataCount = 0;
//     }

//     printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
//   }
//   return 0;
// }




// int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
// {
//   float freq;
//   float phase;

//   fImpPol_Type *pImp = (fImpPol_Type*)pData;
//   AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

//   //printf("Freq:%.2f ", freq);
//   /*Process data*/
//   for(int i=0;i<DataCount;i++)
//   {
//     // printf("Freq:%.2f       ", freq);
//      phase = pImp[i].Phase*180/MATH_PI;
//     if(phase > 180) phase = phase - 360;
//     else if (phase < -180) phase = phase + 360;
//     // printf("RzMag: %f Ohm ,         RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
//     Serial.print(pImp[i].Magnitude);
//     Serial.print(",");
//     Serial.println(phase);
//     // printf("RzMag: %.3f\t",pImp[i].Magnitude);
//     // printf("%.3f\n", phase);

//   }
//   return 0;
// }

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 6;		
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940ImpedanceStructInit(float S_Freq, float E_Freq, int numPoints, BoolFlag logEn)
{
  AppIMPCfg_Type *pImpedanceCfg;
  
  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /* @todo add checker in function */

  pImpedanceCfg->RcalVal = 200.0;
	pImpedanceCfg->FifoThresh = 6;             //
  pImpedanceCfg->SinFreq = 1000.0;          // measurement at 1000 Hz.
	
	/* Configure Excitation Waveform 
	*
  *	 Output waveform = DacVoltPP * ExcitBufGain * HsDacGain 
	* 	
	*		= 300 * 0.25 * 0.2 = 15mV pk-pk
	*
	*/
	pImpedanceCfg->DacVoltPP = 300;	/* Maximum value is 600mV, set up RE on 300mV   */         
	pImpedanceCfg->ExcitBufGain = EXCITBUFGAIN_0P25;
  // pImpedanceCfg->ExcitBufGain = EXCITBUFGAIN_2;
	pImpedanceCfg->HsDacGain = HSDACGAIN_0P2;           // calibration HSDAC.
	
	/* Set switch matrix to onboard(EVAL-AD5940ELECZ) gas sensor. */
	pImpedanceCfg->DswitchSel = SWD_CE0;
	pImpedanceCfg->PswitchSel = SWP_RE0;
	pImpedanceCfg->NswitchSel = SWN_SE0LOAD;
	pImpedanceCfg->TswitchSel = SWT_SE0LOAD;
	/* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is small enough that HSTIA won't be saturated. */
	pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_200;	
	pImpedanceCfg->BiasVolt = 0.0;                      // no DC bias.
  // pImpedanceCfg->AdcPgaGain = ADCPGA_1;           // add gain ADC = 1.
	/* Configure the sweep function. */
	pImpedanceCfg->SweepCfg.SweepEn = bTRUE;
	pImpedanceCfg->SweepCfg.SweepStart = S_Freq;	/* Start from 1kHz */
	pImpedanceCfg->SweepCfg.SweepStop = E_Freq;		/* Stop at 100kHz */
	pImpedanceCfg->SweepCfg.SweepPoints = numPoints;		/* Points is 101 */
	pImpedanceCfg->SweepCfg.SweepLog = logEn;
	/* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
	// pImpedanceCfg->PwrMod = AFEPWR_LP;
  pImpedanceCfg->PwrMod = AFEPWR_HP;
  // pImpedanceCfg->PwrMod = AFEPWR_HP;
	/* Configure filters if necessary */
	pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_4;		/* Sample rate is 800kSPS/2 = 400kSPS */
  pImpedanceCfg->DftNum = DFTNUM_16384;
  pImpedanceCfg->DftSrc = DFTSRC_SINC3;         // calibration DFTSRC.
}

void AD5940_EIS_Main(void)
{
  uint32_t temp;  
  AD5940PlatformCfg();
  AD5940ImpedanceStructInit(S_Vol, E_Vol, StepNumber, logEn);
  
  AppIMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
  AppIMPCtrl(IMPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */

  if (S_Vol == E_Vol)
  {
    timeStart = millis();
    timeNow = 0;
    bool measure = true;
    while(measure)
    {
      if(AD5940_GetMCUIntFlag())
      {
        AD5940_ClrMCUIntFlag();
        temp = APPBUFF_SIZE;
        AppIMPISR(AppBuff, &temp);
        ImpedanceShowResult(AppBuff, temp);
      }
      if (Serial.available())
      {
        char inChar = (char)Serial.read();
        if (inChar == 's')
        {
          measure = false;
        }
      }
    }
  }
  else
  {
    uint32_t count = 0;
    while(count < (StepNumber * RepeatTimes))
    {
      if(AD5940_GetMCUIntFlag())
      {
        AD5940_ClrMCUIntFlag();
        temp = APPBUFF_SIZE;
        AppIMPISR(AppBuff, &temp);
        ImpedanceShowResult(AppBuff, temp);
        count += temp;
      }
    }
  }
}

void setup() {
    
    Serial.begin(115200);
    inputString.reserve(200);
    // // Tạo điểm truy cập cục bộ
    // WiFi.softAP(ssid, password);
    // WiFi.softAPConfig(local_ip, gateway,subnet);
    // delay(100);

    // Serial.println("Access Point started");
    // Serial.print("IP Address: ");
    // Serial.println(WiFi.softAPIP());

    // server.begin();
    // Serial.println("Sever is started");


    uint32_t checkInitMCU = AD5940_MCUResourceInit(0);
    if(checkInitMCU == 0)
    {
    }

}


bool shouldRestart = false;
void loop() {
    while (Serial.available())
  {
    char inChar = (char)Serial.read();
    if (inChar != '!')
    {
      inputString += inChar;
    }
    else
    {
      for(int i = 0; i < inputString.length(); i++)
      {
        if(inputString[i] == '#') {moc1 = i;}
        if(inputString[i] == '?') {moc2 = i;}
        if(inputString[i] == '/') {moc3 = i;}
        if(inputString[i] == '|') {moc4 = i;}
        if(inputString[i] == '$') {moc5 = i;}
      }
      S_Vol = inputString.substring((moc1 + 1), moc2).toDouble() * 1.0;
      E_Vol = inputString.substring((moc2 + 1), moc3).toInt() * 1.0;
      StepNumber = inputString.substring((moc3 + 1), moc4).toInt();
      RepeatTimes = inputString.substring((moc4 + 1), moc5).toInt();
      logEn = (inputString.substring(moc5 + 1).toInt() == 1) ? bTRUE : bFALSE;
      
      if (inputString[0] = '2')
      {
        AD5940_EIS_Main();
      }
      inputString = "";
      ESP.restart();
    }
  }
}