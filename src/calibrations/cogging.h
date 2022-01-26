#ifndef COGGING_H
#define COGGING_H


int FF_COGGING[128]; // [Nm]

/*

This calibration shall compensate the cogging disturbance. 

Goal



*/




/*
 * ODRIVE IMPLEMENTATION
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
// bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
//     float pos_err = input_pos_ - pos_estimate;
//     if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / (float)axis_->encoder_.config_.cpr &&
//         std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / (float)axis_->encoder_.config_.cpr) {
//         config_.anticogging.cogging_map[std::clamp<uint32_t>(config_.anticogging.index++, 0, 3600)] = vel_integrator_torque_;
//     }
//     if (config_.anticogging.index < 3600) {
//         config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
//         input_pos_ = config_.anticogging.index * axis_->encoder_.getCoggingRatio();
//         input_vel_ = 0.0f;
//         input_torque_ = 0.0f;
//         input_pos_updated();
//         return false;
//     } else {
//         config_.anticogging.index = 0;
//         config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
//         input_pos_ = 0.0f;  // Send the motor home
//         input_vel_ = 0.0f;
//         input_torque_ = 0.0f;
//         input_pos_updated();
//         anticogging_valid_ = true;
//         config_.anticogging.calib_anticogging = false;
//         return true;
//     }
// }