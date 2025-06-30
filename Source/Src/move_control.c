#include "move_control.h"



S_handle handle= {0, 0, 0, 0, 0, 0}; // 手柄数据
int16_t go_forward_back = 0;
int16_t go_left_right = 0;
int16_t turn_left_right = 0;
int16_t move_pitch = 0;
int16_t move_row = 0;
int16_t go_up_down = 0;
S_light light = {0, 0};
S_mode mode = {0, 0, 0, 0, 0, 0, 0, 0};
S_pid_depth pid_depth = {0, 0, 0};

//初始化各个pid数据结构体
//为代码简洁方便，用数组表示；分别为俯仰、翻滚、偏航，前后、上下、左右
struct pid_set pid_out_parameter[6];
//8个推进器
struct pid_set pid_in_parameter[8];

struct pid_set depth_pid;

