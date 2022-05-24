#ifndef __CALIBRATEDSENSOR_H__
#define __CALIBRATEDSENSOR_H__

#include "common/base_classes/Sensor.h"
#include "BLDCMotor.h"


class CalibratedSensor: public Sensor{

public:
    // constructor of class with pointer to base class sensor and driver
    CalibratedSensor(Sensor& wrapped);


    /*
    Override the update function
    */
    virtual void update() override;

    /**
    * Calibrate method computes the LUT for the correction
    */
    virtual void calibrate(BLDCMotor& motor);

    float* calibrationLut = new float[n_lut]();
  

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

    // Init inital angles
    float theta_actual { 0 };
    float elecAngle { 0 };
    
    // Hardware related
    int CPR = 16384;                                    // number of counts per revolution --> to be retrieved from Sensor instance
    int NPP = 11;                                       // number of pole pairs --> to be retrieved from BLDC motor --> to be checked            
    int phaseVoltageQ = 6;                              // voltage to run the calibration with
        
    const int n = 128*NPP;                              // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                                  // increments between saved samples (for smoothing motion)
    
    float deltaElectricalAngle = _2PI*NPP/(n*n2);       // Electrical Angle increments for calibration steps    

    // all variables for LUT & calibration yet to be fixed
    const int  n_lut { 128 } ;                             // lut size, currently constant
    float* error_f  = new float[n]();                   // pointer to error array rotating forwards
    float* raw_f = new float[n]();                      // pointer to raw forward position
    float* error_b  = new float[n]();                   // pointer to error array rotating forwards
    float* raw_b = new float[n]();                      // pointer to raw backword position
    float* error = new float[n]();                      // pointer to error array (average of forward & backward)
    float*  error_filt = new float[n]();                // pointer to filtered error array (low pass filter)
    const int window = 128;                             // moving avarage window

    // For while loops to keep track of state --> homing to be added to guarentee the calibration always start at the zero encoder position.
    bool isMeasuring = true;
    bool isHoming = true;

    float elec_angle = 0.0;
    float theta_absolute_post = 0.0;
    float theta_absolute_init = 0.0;
    float theta_init = 0.0;
    float avg_elec_angle = 0.0;
};

#endif