#include "eccentricity.h"

/*

- This calibration shall compensate the misalignment of the encoder magnet and the encoder chip, i.e., the in plane placement error.
- Based on tinymovr approach

- Procedure
    - Incremently move over entire range from 0 to 2PI (Electrical Angle)
    - Store the error (difference between electrical angle and encoder angle)
    - Incremently move back over entire range from 2PI to 0.
    - Store error & take the average of CW and CCW motion.
    - Store encoder error in LUT
    
*/

void WriteLUT(int new_lut[128]){
    memcpy(offset_lut, new_lut, sizeof(offset_lut));
    }
   
void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs){
    /// Measures the electrical angle offset of the position sensor
    /// and (in the future) corrects nonlinearity due to position sensor eccentricity
    printf("Starting calibration procedure\n\r");
    float * error_f;
    float * error_b;
    int * lut;
    int * raw_f;
    int * raw_b;
    float * error;
    float * error_filt;
    
    const int n = 128*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                                                          // increments between saved samples (for smoothing motion)
    float delta = 2*PI*NPP/(n*n2);                                              // change in angle between samples
    error_f = new float[n]();                                                     // error vector rotating forwards
    error_b = new float[n]();                                                     // error vector rotating backwards
    const int  n_lut = 128;
    lut = new int[n_lut]();                                                        // clear any old lookup table before starting.
    
    error = new float[n]();
    const int window = 128;
    error_filt = new float[n]();
    float cogging_current[window] = {0};
    
    ps->WriteLUT(lut); 
    raw_f = new int[n]();
    raw_b = new int[n]();
    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL;                                                             // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    
        
    ///Set voltage angle to zero, wait for rotor position to settle
    motor.setPhaseVoltage(float v_q, float v_d, float theta_ref);
    
    /*
    Rotate Forwards starting from zero position
    */
    printf(" Current Angle : Rotor Angle : Raw Encoder \n\r\n\r");
    for(int i = 0; i<n; i++)
    {                                                 
       for(int j = 0; j<n2; j++)
        {   
            theta_ref += delta;
            motor.setPhaseVoltage(float v_q, float v_d, float theta_ref);
            _micros(100);
        }

        theta_actual = motor.shaft_angle;
        error_f[i] = theta_ref/NPP - theta_actual;
        raw_f[i] = motor.shaft_angle;;
        printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);
    }
    
    /*
    Rotate Backwards starting from the final position of the previous setpoint
    */
    for(int i = 0; i<n; i++)
    {                                                   
       for(int j = 0; j<n2; j++)
        {
            theta_ref -= delta;
            motor.setPhaseVoltage(float v_q, float v_d, float theta_ref); 
            _micros(100);
        }

        theta_actual = motor.shaft_angle;                                // get mechanical position
        error_b[i] = theta_ref/NPP - theta_actual;
        raw_b[i] = motor.shaft_angle;
        printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_b[i]);
    }    
        
    float offset = 0;                                  
    for(int i = 0; i<n; i++)
    {
        offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
    }
        offset = fmod(offset*NPP, 2*PI);                                        // convert mechanical angle to electrical angle
        
            
        ps->SetElecOffset(offset);                                              // Set position sensor offset
        __float_reg[0] = offset;
        E_OFFSET = offset;
        
        /// Perform filtering to linearize position sensor eccentricity
        /// FIR n-sample average, where n = number of samples in one electrical cycle
        /// This filter has zero gain at electrical frequency and all integer multiples
        /// So cogging effects should be completely filtered out.
                
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
            if(i<window){
                cogging_current[i] = current*sinf((error[i] - error_filt[i])*NPP);
                }
            //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
            mean += error_filt[i]/n;
            }
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //Insensitive to errors in this direction, so 2 points is plenty
        
        
        printf("\n\r Encoder non-linearity compensation table\n\r");
        printf(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
        for (int i = 0; i<n_lut; i++){                                          // build lookup table
            int ind = (raw_offset>>7) + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
                }
            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
            printf("%d   %d   %d \n\r", i, ind, lut[ind]);
            wait(.001);
            }
            
        WriteLUT(lut);                                                      // write lookup table to position sensor object
        //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
        
        memcpy(&ENCODER_LUT, lut, 128*4);                                 // copy the lookup table to the flash array
        printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
        //for(int i = 0; i<128; i++){printf("%d\n\r", __int_reg[i]);}
        //printf("\n\r %d \n\r", sizeof(lut));
 
        
        if (!prefs->ready()) prefs->open();
        prefs->flush();                                                         // write offset and lookup table to flash
        prefs->close();
        
        delete[] error_f;       //gotta free up that ram
        delete[] error_b;
        delete[] lut;
        delete[] raw_f;
        delete[] raw_b;
 
    }