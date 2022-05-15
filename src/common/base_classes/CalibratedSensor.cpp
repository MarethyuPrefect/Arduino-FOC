#include "CalibratedSensor.h"


// CalibratedSensor()
// sensor              - instance of original sensor object
// driver              - instance of the driver object


CalibratedSensor::CalibratedSensor(Sensor& wrapped) : _wrapped(wrapped) {
};

void CalibratedSensor::update(){
    _wrapped.update();
    this->Sensor::update();
};

void Sensor::init() {
    // assume wrapped sensor has already been initialized
    this->Sensor::init(); // call superclass init
}

float CalibratedSensor::getSensorAngle(){
    // raw encoder count e.g. 0-16384 (14 bits encoder) 
    int rawCount = _wrapped.getMechanicalAngle();

    // only use the lower 7 bits (0-128) 
    int lowerBits = (rawCount&0x007F);
    
    // Extract the lower and upper LUT value in counts
    int y0 = calibrationLut[rawCount>>7]; 
    int y1 = calibrationLut[((rawCount>>7)+1)%128]; 
    
    // Linear Interpolation Between LUT values y0 and y1. 
    // If lowerBits = 0, interpolated offset = y0
    // If lowerBits = 1, interpolated offset = y1
    float interpolatedOffset = ((127.0f-lowerBits)/127.0f)*y0 + (lowerBits/127.0f)*y1; 

    // add offset to the raw sensor count. Divide multiply by 2PI/CPR to get radians
    float calibratedAngle = (((rawCount+interpolatedOffset)/(float)CPR) * _2PI) ; 

    // return calibrated angle in radians
    return calibratedAngle;
}

void CalibratedSensor::calibrate(BLDCMotor& motor){
    float elecAngle = 0;
    bool isMeasuring = true;
    int phaseVoltageQ = 4;
    
    // start with zero offset
    motor.zero_electric_angle = 0; // Set position sensor offset
   ///Set voltage angle to zero, wait for rotor position to settle
    for(int i = 0; i<40000; i++){
        motor.setPhaseVoltage(phaseVoltageQ, 0, elecAngle);
         _delay(0.1);
    }

// Loop over different electrical angles, once forward, once backward
// store actual position and error as compared to electrical angle
while(isMeasuring)
    {
    // forward rotation
    Serial.println("Rotating forwards");
    for(int i = 0; i<n; i++)
    {                                                 
       for(int j = 0; j<n2; j++)
        {   
          elecAngle += deltaElectricalAngle;
          motor.setPhaseVoltage(phaseVoltageQ, 0, elecAngle);
          _delay(1);
        }
        sensor.update();
        theta_actual = sensor.getUncalibratedAngle();
        error_f[i] = elecAngle/pole_pairs - theta_actual;
        raw_f[i] = theta_actual;

        // only temporary printing for debugging
        Serial.print(elecAngle/(pole_pairs),5);
        Serial.print("\t");
        Serial.print(theta_actual,5);
        Serial.print("\t");
        Serial.println(error_f[i],5);
    }

    _delay(2000);

    // backwards rotation
    Serial.println("Rotating backwards");
    for(int i = 0; i<n; i++)
    {                                                 
       for(int j = 0; j<n2; j++)
        {   
          elecAngle -= deltaElectricalAngle;
          motor.setPhaseVoltage(phaseVoltageQ, 0 ,elecAngle);
          _delay(1);
        }
        sensor.update();
        theta_actual = sensor.getUncalibratedAngle();
        error_b[i] = elecAngle/pole_pairs - theta_actual;
        raw_b[i] = theta_actual;

        // only temporary printing for debugging
        Serial.print(elecAngle/(pole_pairs),5);
        Serial.print("\t");
        Serial.print(theta_actual,5);
        Serial.print("\t");
        Serial.println(error_b[i],5);
    }
    // done with the measurement
    isMeasuring=false;
    motor.setPhaseVoltage(0, 0, 0);

    }

    float offset = 0;     
     // calclate average position sensor offset                             
    for(int i = 0; i<n; i++){
        offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                  
    }
    // convert mechanical angle to electrical angle
    offset = fmod(offset*NPP, 2*PI);                                        
        
    // Set the avarage electrical offset. Around this avarage offset the LUT will compensante
    motor.zero_electric_angle = offset;                            
    Serial.println("Average Electrical Angle Offset: ");
    Serial.println(offset);

    /// Perform filtering to linearize position sensor eccentricity
    /// FIR n-sample average, where n = number of samples in one electrical cycle
    /// This filter has zero gain at electrical frequency and all integer multiples
    /// So cogging effects should be completely filtered out
        
    // still need to understand this function filtering. I've removed a cogging variable that was not used elsehwere in the code.
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
    int raw_offset = (raw_f[0] + raw_b[n-1])/2;                   
 

    Serial.println(" Encoder non-linearity compensation table");
    Serial.println(" Lookup Index : Lookup Value");
    // Build Look Up Table
    for (int i = 0; i<n_lut; i++){                                          
        int ind = (raw_offset>>7) + i;
        if(ind > (n_lut-1)){ 
            ind -= n_lut;
        }
        lut[ind] = (int) ((error_filt[i*NPP] - mean)*(CPR/(2.0f*PI)));
        Serial.print(ind);
        Serial.print('\t');
        Serial.println(lut[ind]);
        _delay(1);
    }

    // how does this LUT end up in the getSensorAngle function???
    writeLut(lut);
}

void writeLUT(int newLut[128]){

    // what is the use of this function? Why can I not just pass 'lut' from the doCalibration function?
    memcpy(calibrationLut, newLut, sizeof(calibrationLut));
}

