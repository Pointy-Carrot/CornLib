#pragma once

namespace cornlib{

/**
 * @brief struct to hold PID gains
 * 
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
struct Gains{
        float kP = 0;
        float kI = 0;
        float kD = 0;
};

class PID{
    private:
        Gains m_gains;

        bool m_sign_flip_reset;
        float m_windup_range;
        float m_prev_error;
        float m_error;
        float m_integral;

        float m_previous_time;


    public:
        /**
         * @brief PID constructor
         * 
         * @param kP proportional tuning value
         * @param kI integral tuning value
         * @param kD derivative tuning value
         * @param windup_range range where integral is reset
         * @param sign_flip_reset whether integral should reset when error changes sign
         */
        PID(float kP, float kI, float kD, float windup_range = 0, bool sign_flip_reset = false);

        /**
         * @brief Get the current gains
         * 
         * @return Gains the current gains
         */
        Gains get_gains();
        
        /**
         * @brief Set the new gains
         * 
         * @param gains new gains
         */
        void set_gains(Gains gains);

        /**
         * @brief Updates the PID using the current error and outputs the PID calculation
         * 
         * @param error current error
         * @return output power
         */
        float update(float error);

        /**
         * @brief resets the values of the PID controller
         * 
         */
        void reset();

        /**
         * @brief change whether the integral is reset when the error changes sign
         * 
         * @param sign_flip_reset whether to reset the integral when the error changes sign
         */
        void sign_flip_reset(bool sign_flip_reset);

        /**
         * @brief Get the sign_flip_reset value
         * 
         * @return true sign_flip_reset is true
         * @return false sign_flip_reset is false
         */
        bool get_sign_flip_reset();

        /**
         * @brief Set the new windup range
         * 
         * @param windup_range new windup range
         */
        void set_windup_range(float windup_range);

        /**
         * @brief Get the windup range value
         * 
         * @return current windup range
         */
        float get_windup_range();

};
}