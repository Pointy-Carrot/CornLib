#include "CornLib/PID.h"
#include "Cornlib/robot_init.h"

namespace cornlib{

cornlib::PID::PID(float kP, float kI, float kD, float windup_range = 0, bool sign_flip_reset = false) :
m_gains({kP, kI, kD}), m_windup_range(windup_range), m_sign_flip_reset(sign_flip_reset) {};

Gains cornlib::PID::get_gains(){
    return {m_gains.kP, m_gains.kI, m_gains.kD};
}

void cornlib::PID::set_gains(Gains gains){
    m_gains.kP = gains.kP;
    m_gains.kI = gains.kI;
    m_gains.kD = gains.kD;
}

float cornlib::PID::update(float error){
    // capture time
    const Time now = Brain.Timer.value();
    Time dt = (m_previous_time == std::nullopt) ? 0 : now - *m_previous_time;
    m_previous_time = now;

    // calculate derivative
    const float deriv = (dt != 0) ? (error - m_prev_error) / (dt/1000) : 0;
    m_prev_error = error;

    // calculate integral
    m_integral += error * (dt/1000);

    // integral resets
    if((((error < 0) && (m_prev_error > 0)) || ((error > 0) && (m_prev_error < 0))) && sign_flip_reset){
        m_integral = 0;
    }
    if((abs(error) > m_windup_range) && (m_windup_range != 0)){
        m_integral = 0;
    }

    // return calculations
    return error*m_gains.kP + m_integral*m_gains.kI + deriv*m_gains.kD;
}

void cornlib::PID::reset(){
    m_prev_error = 0;
    m_integral = 0;
}

void cornlib::PID::sign_flip_reset(bool sign_flip_reset){
    m_sign_flip_reset = sign_flip_reset;
}

bool cornlib::PID::get_sign_flip_reset(){
    return m_sign_flip_reset;
}

void cornlib::PID::set_windup_range(float windup_range){
    m_windup_range = windup_range;
}

float cornlib::PID::get_windup_range(){
    return m_windup_range;
}

} // namespace cornlib