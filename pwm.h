#ifndef PWM_H_
#define PWM_H_

void FTM0_init(void);
void FTM0_set_duty_cycle(unsigned int duty_cycle, unsigned int frequency, int dir,int motor);
void FTM3_init(void);
void FTM3_set_duty_cycle(float duty_cycle, unsigned int frequency, int dir);
void en_motors(void);
void init_motors(void);



#endif /* PWM_H_ */
