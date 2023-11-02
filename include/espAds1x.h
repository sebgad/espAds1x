#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include <algorithm>
#include "espAds1x_defines.h"

class ADS1x
{
    public:
        ADS1x();
        esp_err_t stop(void);
        esp_err_t writeConfiguration(void);
        void startSingleShotMeas();
        bool getOpStatus(void);
        void setMux(uint8_t);
        uint8_t getMux(void);
        void setPGA(uint8_t);
        uint8_t getPGA(void);
        void setOpMode(bool);
        uint8_t getOpMode(void);
        void setRate(uint8_t);
        uint8_t getRate(void);
        void setCompMode(bool);
        uint8_t getCompMode(void);
        void setCompPolarity(bool);
        uint8_t getCompPolarity(void);
        void setCompLatchingMode(bool);
        uint8_t getCompLatchingMode(void);
        void setCompQueueMode(uint8_t);
        uint8_t getCompQueueMode(void);
        void setCompLowThreshBit(bool, int);
        uint8_t getCompLowThreshBit(int);
        void setCompHighThreshBit(bool, int);
        uint8_t getCompHighThreshBit(int);
        void setPinRdyMode(uint8_t);
        uint8_t getPinRdyMode(void);
        esp_err_t readConversionRegister(uint16_t *);
        bool isValueFrozen(void);
        esp_err_t getConvVal(float *);
        esp_err_t getVoltVal(float *);
        esp_err_t getPhysVal(float *);
        int getLatestBufVal(void);
        void printConfigReg(void);
        esp_err_t getRegisterValue(uint8_t, uint16_t*);
        esp_err_t setRegisterValue(uint8_t, uint16_t);
        void setPhysConv(const float, const float);
        void setPhysConv(const float, const float, const float);
        void setPhysConv(const float[][2], size_t);
        void activateFilter();
        void deactivateFilter();
        bool getFilterStatus(void);
        int getAbsBufSize(void);
        int16_t* getBuffer(void);
        void getGradientPhysVal(float *, float * );

    private:
        QueueHandle_t _conv_queue;
        uint16_t _iState;
        uint16_t _iError;
        float ** _ptrConvTable;
        size_t _iSizeConvTable;
        uint16_t _iConfigRegister;
        uint16_t _iLowThreshRegister;
        uint16_t _iHighThreshRegister;
        esp_err_t _initI2CMaster();
        void _initConvTable(size_t);
        void _writeBit(uint16_t &, int, bool);
        bool _readBit(uint16_t, int);
        int _iBuffCnt;
        int _iBuffMaxFillIndex;
        int16_t * _ptrConvBuff;
        float * _ptrFilterCoeff;
        float _fFilterNormCoeff;
        bool _bFilterActive;
        bool _bSavGolFilterActive;
        float _getAvgFilterVal();
        float _getSavGolFilterVal();
        int _iValFroozenDebCnt;
        uint8_t _lowbyte(uint16_t);
        uint8_t _highbyte(uint16_t);
};