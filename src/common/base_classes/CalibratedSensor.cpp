#include "CalibratedSensor.h"


// CalibratedSensor()
// sensor              - instance of original sensor object
CalibratedSensor::CalibratedSensor(Sensor& wrapped) : _wrapped(wrapped) 
{
};

// call update of calibrated sensor
void CalibratedSensor::update(){
    _wrapped.update();
    this->Sensor::update();
};

// Retrieve the calibrated sensor angle
void CalibratedSensor::init() {
    // assume wrapped sensor has already been initialized
    this->Sensor::init(); // call superclass init
}

// Retrieve the calibrated sensor angle
float CalibratedSensor::getSensorAngle(){
    // raw encoder position e.g. 0-2PI
    float rawAngle = _wrapped.getMechanicalAngle();

    // index of the bucket that rawAngle is part of. 
    // e.g. rawAngle = 0 --> bucketIndex = 0.
    // e.g. rawAngle = 2PI --> bucketIndex = 128.
    int bucketIndex = floor(rawAngle/(_2PI/n_lut));
    float remainder = rawAngle - ((_2PI/n_lut)*bucketIndex);

    // Extract the lower and upper LUT value in counts
    float y0 = calibrationLut[bucketIndex]; 
    float y1 = calibrationLut[(bucketIndex+1)%128]; 
    
    // Linear Interpolation Between LUT values y0 and y1 using the remainder
    // If remainder = 0, interpolated offset = y0
    // If remainder = 2PI/n_lut, interpolated offset = y1
    float interpolatedOffset = (((_2PI/n_lut)-remainder)/(_2PI/n_lut))*y0 + (remainder/(_2PI/n_lut))*y1; 

    // add offset to the raw sensor count. Divide multiply by 2PI/CPR to get radians
    float calibratedAngle = rawAngle+interpolatedOffset; 

    //Serial.print(rawAngle,5);
    //Serial.print("\t");
    //Serial.println(calibratedAngle,5);

    // return calibrated angle in radians
    return calibratedAngle;
}

void CalibratedSensor::calibrate(BLDCMotor& motor){
    
    Serial.println("Starting Sensor Calibration.");

    // start with zero offset
    motor.zero_electric_angle = 0; // Set position sensor offset
    
    //Set voltage angle to zero, wait for rotor position to settle
    for(int i = 0; i<40000; i++)
        {
            motor.setPhaseVoltage(phaseVoltageQ, 0, elec_angle);
        }

    // keep the motor in position while getting the initial positions
    motor.setPhaseVoltage(phaseVoltageQ, 0, elec_angle);
    _delay(1000);
    _wrapped.update();
    float theta_init = _wrapped.getAngle();
    float theta_absolute_init = _wrapped.getMechanicalAngle();

    /* 
    
    Start Calibration
    Loop over  electrical angles from 0 to NPP*2PI, once forward, once backward
    store actual position and error as compared to electrical angle
    
    */

    while(isMeasuring)
        {

        /* 
        forwards rotation
        */
        Serial.println("Rotating forwards");
        int k = 0;
        for(int i = 0; i<n; i++)
        {                                                 
            for(int j = 0; j<n2; j++)
                {   
                    elec_angle += deltaElectricalAngle;
                    motor.setPhaseVoltage(phaseVoltageQ, 0, elec_angle);
                }
            
            // delay to settle in position before taking a position sample
            _delay(20);
            _wrapped.update();
            theta_actual = _wrapped.getAngle()-theta_init;
            // if overflow happened track it as full rotation
            error_f[i] = elec_angle/NPP - theta_actual;
            raw_f[i] = theta_actual;

            // storing the normalized angle every time the electrical angle 3PI/2 to calculate average zero electrical angle
            if(i==(k*128+96))
                {
                    _delay(500);
                    avg_elec_angle += _normalizeAngle(_wrapped.getMechanicalAngle()*NPP);
                    k += 1;
                }
            
            // only temporary printing for debugging
            Serial.print(elec_angle/(NPP),5);
            Serial.print("\t");
            Serial.print(theta_actual,5);
            Serial.print("\t");
            Serial.println(error_f[i],5);
        }
            

        _delay(2000);

        /* 
        backwards rotation
        */
        Serial.println("Rotating backwards");
        for(int i = 0; i<n; i++)
        {                                                 
            for(int j = 0; j<n2; j++)
                {   
                    elec_angle -= deltaElectricalAngle;
                    motor.setPhaseVoltage(phaseVoltageQ, 0 ,elec_angle);
                }

                // delay to settle in position before taking a position sample
                _delay(20);
                _wrapped.update();
                theta_actual = _wrapped.getAngle()-theta_init;
                error_b[i] = elec_angle/NPP - theta_actual;
                raw_b[i] = theta_actual;

                // only temporary printing for debugging
                Serial.print(elec_angle/(NPP),5);
                Serial.print("\t");
                Serial.print(theta_actual,5);
                Serial.print("\t");
                Serial.println(error_b[i],5);
        }

        _wrapped.update();
        theta_absolute_post = _wrapped.getMechanicalAngle();

        // done with the measurement
        isMeasuring=false;
        motor.setPhaseVoltage(0, 0, 0);

        }

        // raw offset from initial position in absolute radians between 0-2PI
        float raw_offset = (theta_absolute_init+theta_absolute_post)/2;                   

        // calculating the average zero electrica angle from the forward calibration.
        motor.zero_electric_angle  = avg_elec_angle/NPP;
        Serial.print( "Average Zero Electrical Angle: ");
        Serial.println( motor.zero_electric_angle); 
        
        /// Perform filtering to linearize position sensor eccentricity
        /// FIR n-sample average, where n = number of samples in one electrical cycle
        /// This filter has zero gain at electrical frequency and all integer multiples
        /// So cogging effects should be completely filtered out
            
        float mean = 0;
        for (int i = 0; i<n; i++){                                              //Average the forward and back directions
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
            }
        for (int i = 0; i<n; i++){
            for(int j = 0; j<window; j++){
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0){
                    ind += n;}                                                  // Moving average wraps around
                else if(ind > n-1) {
                    ind -= n;}
                error_filt[i] += error[ind]/(float)window;
            }
            mean += error_filt[i]/n;
        }

        // calculate offset index
        int index_offset = floor(raw_offset/(_2PI/n_lut));

        Serial.println(" Encoder non-linearity compensation table");
        Serial.println(" Lookup Index : Lookup Value");
        // Build Look Up Table
        for (int i = 0; i<n_lut; i++){                                          
            int ind =  index_offset + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
            }
            calibrationLut[ind] = (float) (error_filt[i*NPP] - mean);
            Serial.print(ind);
            Serial.print('\t');
            Serial.println(calibrationLut[ind],5);
            _delay(1);
        }
   
   
    Serial.println("Sensor Calibration Done.");


}



