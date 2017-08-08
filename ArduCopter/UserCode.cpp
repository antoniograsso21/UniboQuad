#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
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
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
#include <SRV_Channel/SRV_Channel.h>
#define PI 3.14159265358979323846 // pi definition
#define DELTA_TA 0.2 // parametro delta_Ta

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
void move_servo(int channel, int angle_value, int angle_min, int angle_max, int output_min, int output_max)
{
    /*
     * Ottengo il canale passato come parametro:
     * AUX_CH 1 == channels[8]
     * AUX_CH 2 == channels[9]
     * AUX_CH 3 == channels[10]
     * AUX_CH 4 == channels[11]
     */
    SRV_Channel * ch = SRV_Channels::srv_channel(channel);

    // definisco dei valori di interesse
    const int rangeIn = angle_max - angle_min;
    const int rangeOut = output_max - output_min;
    const int deltaIn = angle_value - angle_min;

    // definisco costanti matematiche per migliorare l'accuratezza del risultato di divisioni ed approssimazioni
    const int fixedHalfDecimal = 1;
    const int fixedDecimal = fixedHalfDecimal * 2;

    // calcolo il valore pwm da scrivere sul canale per muovere il servo motore
    uint16_t pwm_value = ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + output_min;

    // scrivo sul canale il valore pwm precedentemente calcolato
    ch->set_output_pwm(pwm_value);
}

// 3.3Hz code
void Copter::userhook_SlowLoop()
{
    int16_t phi = 90;
    // ottengo la distanza dal terreno calcolata dal rangefinder (cm)
    uint16_t rangefinder_distance = rangefinder.distance_cm(0); // l
    bool is_rf_ok = true;

    // controlla quanti rangefinders siano connessi alla pixhawk e la distanza misurata dal rangefinder
    if(rangefinder.num_sensors() != 1 || rangefinder_distance == 0) {
        is_rf_ok = false;
    }

    /*
     * Codice eseguito se è connesso un rangefinder alla
     * pixhawk che misura una distanza positiva dal terreno
     */
    if(is_rf_ok) {

        // ottengo la velocità secondo ENU (East North Up)
        const Vector3f &velocity = inertial_nav.get_velocity();
        // get velocity x according to body frame (cm/s) (along roll axis)

        // ottengo la variabile di attitude heading reference system
        AP_AHRS_DCM ahrs_dcm = ahrs;
        // calcolo la velocità orizzontale (vx) secondo l'asse corpo (cm/s)
        float vxb = (velocity.y*ahrs_dcm.cos_pitch()*ahrs_dcm.cos_yaw() - velocity.x*ahrs_dcm.cos_pitch()*ahrs_dcm.sin_yaw() - velocity.z*ahrs_dcm.sin_pitch());

        // ottengo l'angolo di pitch (gradi); (pitch_sensor: gradi * 100)
        int16_t theta = (ahrs_dcm.pitch_sensor)/100;

        // calcolo lo spazio di anticipo: delta_Ta * Vx (cm)
        float lead_space = DELTA_TA * vxb; // Sa

        // calcolo l'angolo delta: arcsin(Sa/l) * 180 / PI
        int16_t delta = ((asin(lead_space/rangefinder_distance)) * 180) / PI;

        // calcolo l'angolo finale della rotazione (gradi)
        phi = theta + delta;

    }

    // Obbligo l'angolo ad avere un valore compreso tra 0 e 180 (nel caso non lo sia)
    phi = constrain_int16(phi,0,180);

    // 544 pulsazione minima; 2375 pulsazione massima: valori ricavati empiricamente
    // Per muovere il servo di un angolo x bisogna modificare solo il secondo parametro impostandolo a x
    move_servo(9,phi,0,180,544,2375);
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
