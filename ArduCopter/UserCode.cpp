#include "Copter.h"

/*
 * @brief This function allows to move the servo connected to the Pixhawk (reimplementation of Arduino function)
 * @params
 * channel: Pixhawk's channel which the servo is connected to
 * angle_value: angle value which describes servo rotation
 * angle_min: infimum angle value
 * angle_max: supremum angle value
 * output_min: min pulse width
 * output_max: max pulse width
 */
static void move_servo(int channel, int angle_value, int angle_min, int angle_max, int output_min, int output_max)
{
    // check first parameter
    if(channel < 0 || channel > 13 ) // 14 canali OUT
        return;
    // values of interests
    const int rangeIn = angle_max - angle_min;
    const int rangeOut = output_max - output_min;
    const int deltaIn = angle_value - angle_min;

    const int fixedHalfDecimal = 1;
    const int fixedDecimal = fixedHalfDecimal * 2; //2

    // pwm value used to move servo
    uint16_t pwm_value = ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + output_min;

    /*
     * AUX_CH 1 == channels[8]
     * AUX_CH 2 == channels[9]
     * AUX_CH 3 == channels[10]
     * AUX_CH 4 == channels[11]
     */
    // write the desired pwm value to the channel which the servo is connected to
    SRV_Channels::srv_channel(channel)->set_output_pwm(pwm_value);
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put here your inizialization code
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
#define PI 3.14159265358979323846 // definizione di pi greco
#define DELTA_TA 0.2 // parametro DELTA_Ta
#define H_REF 150 // parametro h*, altezza di riferimento
void Copter::userhook_MediumLoop()
{
    // attitude heading reference system
    AP_AHRS_DCM ahrs_dcm = ahrs;

    AP_InertialNav_NavEKF in_nav = inertial_nav;
    // inertial velocity
    const Vector3f &velocity = in_nav.get_velocity();

    // sb vector's components through PX4Flow algorithm (when using PX4Flow)
    double sxb = ahrs_dcm.cos_pitch()*velocity.x*DELTA_TA - ahrs_dcm.sin_pitch()*H_REF;
    double syb = ahrs_dcm.sin_roll()*ahrs_dcm.sin_pitch()*velocity.x*DELTA_TA
            + ahrs_dcm.cos_roll()*velocity.y*DELTA_TA
            + ahrs_dcm.sin_roll()*ahrs_dcm.cos_pitch()*H_REF;
    double szb = ahrs_dcm.cos_roll()*ahrs_dcm.sin_pitch()*velocity.x*DELTA_TA
            - ahrs_dcm.sin_roll()*velocity.y*DELTA_TA
            + ahrs_dcm.cos_roll()*ahrs_dcm.cos_pitch()*H_REF;

    // delta in radians
    double delta_radians = 0;
    if(szb == 0)
        szb = 0.01;
    delta_radians = atan(sxb/szb);

    // delta in degrees: (delta_radians * 180) / PI
    int16_t delta_degrees = (delta_radians * 180) / PI;

    // value used to get lambda
    double ratio = sin(delta_radians)*sxb + cos(delta_radians)*szb;

    // lambda in radians
    double lambda_radians = 0;
    if(ratio == 0)
        ratio = 0.01;
    lambda_radians = - atan(syb/ratio);

    // lambda in degrees: (lambda_radians * 180) / PI
    int16_t lambda_degrees = (lambda_radians * 180) / PI;

    // rotation angle of the first servo
    delta_degrees = 90 + delta_degrees;
    // rotation angle of the second servo
    lambda_degrees = 90 + lambda_degrees;

    // constraint for angles to have a value between 0 and 180 degrees
    delta_degrees = constrain_int16(delta_degrees,0,180);
    lambda_degrees = constrain_int16(lambda_degrees,0,180);

    // Move both servos
    move_servo(9,delta_degrees,0,180,715,2345);
    move_servo(10,lambda_degrees,0,180,590,2360);

    /* ------------------ DEBUG --------------------
       hal.console->println("----------------------");
       hal.console->print("------ Velocity x --- ");
       hal.console->println(velocity.x);
       hal.console->print("------ Velocity y --- ");
       hal.console->println(velocity.y);
       hal.console->print("--- -- Velocity z --- ");
       hal.console->println(velocity.z);


       hal.console->print("--- -- sxb --- ");
       hal.console->println(sxb);
       hal.console->print("--- -- syb --- ");
       hal.console->println(syb);
       hal.console->print("--- -- szb --- ");
       hal.console->println(szb);

       hal.console->print("--- Delta radians --- ");
       hal.console->println(delta_radians);

       hal.console->print("------- Pitch ------- ");
       hal.console->println(ahrs_dcm.pitch_sensor/100);
       hal.console->print("--- Delta degrees --- ");
       hal.console->println(delta_degrees);

       hal.console->print("-------- Ratio ------ ");
       hal.console->println(ratio);
       hal.console->print("--- Lambda radians -- ");
       hal.console->println(lambda_radians);

       hal.console->print("--------- Roll -------- ");
       hal.console->println(ahrs_dcm.roll_sensor/100);
       hal.console->print("--- Lambda degrees ---- ");
       hal.console->println(lambda_degrees);
       */

}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // 3.3Hz code
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{

}
#endif
