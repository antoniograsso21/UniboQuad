#include "Copter.h"

/*
 * @brief Questa funzione muove un servo motore collegato alla pixhawk secondo l'angolo indicato
 * @params
 * channel: canale della pixhawk a cui è collegato il servo motore
 * angle_value: valore dell'angolo a cui si desidera impostare il servo motore
 * angle_min: angolo estremo inferiore della rotazione del servo motore
 * angle_max: angolo estremo superiore della rotazione del servo motore
 * output_min: min pulse width
 * output_max: max pulse width
 */
static void move_servo(int channel, int angle_value, int angle_min, int angle_max, int output_min, int output_max)
{
    // Controllo la correttezza del primo parametro
    if(channel < 0 || channel > 13 ) // 14 canali OUT
        return;
    // definisco dei valori di interesse
    const int rangeIn = angle_max - angle_min; // 180
    const int rangeOut = output_max - output_min; //900
    const int deltaIn = angle_value - angle_min; //20

    // definisco costanti matematiche per migliorare l'accuratezza del risultato di divisioni ed approssimazioni
    const int fixedHalfDecimal = 1;
    const int fixedDecimal = fixedHalfDecimal * 2; //2

    // calcolo il valore pwm da scrivere sul canale per muovere il servo motore
    uint16_t pwm_value = ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + output_min;

    /*
     * AUX_CH 1 == channels[8]
     * AUX_CH 2 == channels[9]
     * AUX_CH 3 == channels[10]
     * AUX_CH 4 == channels[11]
     */
    // scrivo sul canale desiderato il valore pwm precedentemente calcolato
    SRV_Channels::srv_channel(channel)->set_output_pwm(pwm_value); //line 397 SRV_Channel.h
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
    // Ottengo l'attributo di attitude heading reference system
    AP_AHRS_DCM ahrs_dcm = ahrs;

    AP_InertialNav_NavEKF in_nav = inertial_nav;
    // Ottengo la velocità inerziale
    const Vector3f &velocity = in_nav.get_velocity();

    // Calcolo le 3 componenti del vettore sb secondo algoritmo PX4Flow
    double sxb = ahrs_dcm.cos_pitch()*velocity.x*DELTA_TA - ahrs_dcm.sin_pitch()*H_REF;
    double syb = ahrs_dcm.sin_roll()*ahrs_dcm.sin_pitch()*velocity.x*DELTA_TA
            + ahrs_dcm.cos_roll()*velocity.y*DELTA_TA
            + ahrs_dcm.sin_roll()*ahrs_dcm.cos_pitch()*H_REF;
    double szb = ahrs_dcm.cos_roll()*ahrs_dcm.sin_pitch()*velocity.x*DELTA_TA
            - ahrs_dcm.sin_roll()*velocity.y*DELTA_TA
            + ahrs_dcm.cos_roll()*ahrs_dcm.cos_pitch()*H_REF;

    // Calcolo delta in radianti secondo l'algoritmo
    double delta_radians = 0;
    if(szb == 0)
        szb = 0.01;
    delta_radians = atan(sxb/szb);

    // Calcolo l'angolo delta in gradi: (delta_radians * 180) / PI
    int16_t delta_degrees = (delta_radians * 180) / PI;

    // Calcolo il denominatore da cui ricavare lambda
    double ratio = sin(delta_radians)*sxb + cos(delta_radians)*szb;

    // Calcolo lambda in radianti secondo l'algoritmo
    double lambda_radians = 0;
    if(ratio == 0)
        ratio = 0.01;
    lambda_radians = - atan(syb/ratio);

    // Calcolo l'angolo lambda in gradi: (lambda_radians * 180) / PI
    int16_t lambda_degrees = (lambda_radians * 180) / PI;

    // Calcolo l'angolo di rotazione del servo 1
    delta_degrees = 90 + delta_degrees;
    // Calcolo l'angolo di rotazione del servo 2
    lambda_degrees = 90 + lambda_degrees;

    // Obbligo gli angoli ad avere un valore compreso tra 0 e 180 (nel caso non in cui non lo fossero)
    delta_degrees = constrain_int16(delta_degrees,0,180);
    lambda_degrees = constrain_int16(lambda_degrees,0,180);

    // Muovo i due servomotori degli angoli desiderati
    // Per muovere il servo di un angolo x (gradi) bisogna modificare
    // solo il secondo parametro, impostandolo a x
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
    // 1Hz code
}
#endif
