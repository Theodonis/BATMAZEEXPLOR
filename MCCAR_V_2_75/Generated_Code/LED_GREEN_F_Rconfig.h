#ifndef __LED_GREEN_F_R_CONFIG_H
#define __LED_GREEN_F_R_CONFIG_H

#ifndef LED_GREEN_F_R_CONFIG_USE_GPIO_PIN
  #define LED_GREEN_F_R_CONFIG_USE_GPIO_PIN   (1)
    /*!< 1: use GPIO pin; 0: use PWM pin */
#endif

#ifndef LED_GREEN_F_R_CONFIG_IS_LOW_ACTIVE
  #define LED_GREEN_F_R_CONFIG_IS_LOW_ACTIVE   (1)
    /*!< 1: LED is low active (cathode on port side), 0: LED is HIGH active (anode on port side) */
#endif

#endif /* __LED_GREEN_F_R_CONFIG_H */
