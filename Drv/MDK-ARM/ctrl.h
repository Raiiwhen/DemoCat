#ifndef __CTRL_H
#define __CTRL_H

void ctrl_init(void);
void ctrl_exe(void);
void encoder3_update(void);
void encoder4_update(void);
float get_mr_cnt(void);
float get_ml_cnt(void);
float get_mr_vel(void);
float get_ml_vel(void);

#endif
