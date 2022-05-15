#ifndef CALIBRATESENSOR_H
#define CALIBRATESENSOR_H

class CalibrateSensor{

public:
    // constructor of class with pointer to base class sensor and driver
    CalibrateSensor(Sensor* sensor);
    
    // This function returns the interpolated angle after calibration
    float getSensorAngle() override;

    // this function will return the uncalibrated value
    float getUncalibratedAngle();

    void init() override;

    // making this a friend class such that we can use the setPhaseVoltage


private:
    // execute calibration (move +2PI, -2PI), use function to writeLUT.
    void doCalibration();

    // fills the Look Up Table 
    void writeLut();

    const int  n_lut = 128;
    calibrationLut = new int[n_lut]();   
    float theta_actual;
    float elecAngle = 0;
    int NPP = motor.pole_pairs;
    const int n = 128*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                                                          // increments between saved samples (for smoothing motion)
    float deltaElectricalAngle = 2*PI*NPP/(n*n2);      
    float* error_f  = new float[n]();                                                     // error vector rotating forwards
    int* raw_f = new int[n]();  
    float* error_b  = new float[n]();                                                     // error vector rotating forwards
    int* raw_b = new int[n]();  
    float* error = new float[n]();
    const int window = 128;
    float*  error_filt = new float[n]();
    float cogging_current[window] = {0};  
    int CPR = 16384;

    float elecAngle = 0;
    bool isMeasuring = true;
    int phaseVoltageQ = 4;
    

    bool isMeasuring = true;
    bool isHoming = true;
    bool isCalibrating = true;


}

#endif