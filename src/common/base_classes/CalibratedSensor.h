#ifndef __CALIBRATEDSENSOR_H__
#define __CALIBRATEDSENSOR_H__

#include "common/base_classes/Sensor.h"
#include "BLDCMotor.h"


class CalibratedSensor: public Sensor{

public:
    // constructor of class with pointer to base class sensor and driver
    CalibratedSensor(Sensor& wrapped);

    virtual void update() override;

    /**
    * Calibrate method computes the LUT for the correction
    */
    virtual void calibrate(BLDCMotor& motor);

    // all variables for LUT & calibration yet to be fixed
    const int  n_lut = 128;
    int* calibrationLut = new int[n_lut]();   
    bool isMeasuring = true;
    float theta_actual;
    float elecAngle = 0;
    int NPP = 11;
    const int n = 128*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                                                          // increments between saved samples (for smoothing motion)
    float deltaElectricalAngle = _2PI*NPP/(n*n2);      
    float* error_f  = new float[n]();                                                     // error vector rotating forwards
    int* raw_f = new int[n]();  
    float* error_b  = new float[n]();                                                     // error vector rotating forwards
    int* raw_b = new int[n]();  
    float* error = new float[n]();
    const int window = 128;
    float*  error_filt = new float[n]();
    int CPR = 16384;
    bool isHoming = true;
    bool isCalibrating = true;

protected:

    /**
    * getSenorAngle() method of CalibratedSensor class.
    * This should call getAngle() on the wrapped instance, and then apply the correction to
    * the value returned. 
    */
    virtual float getSensorAngle() override;
    /**
    * init method of CaibratedSensor - call after calibration
    */
    virtual void init() override;
    /**
    * delegate instance of Sensor class
    */
    Sensor& _wrapped;

};

#endif