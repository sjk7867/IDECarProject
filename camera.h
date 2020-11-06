#ifndef CAMERA_H
#define CAMERA_H

void ADC_IRQHandler(void);
void FTM2_IRQHandler(void);
void PIT0_IRQHandler(void);
void init_FTM2(void);
void init_PIT(void);
void init_GPIO(void);
void init_ADC0(void);
void init_camera(void);
void getline(uint16_t *newline);
void line_averager(void);
void edge_finder(void);
int maxsmooth(void);
int minsmooth(void);
int leftedge(void);
int rightedge(void);
#endif
