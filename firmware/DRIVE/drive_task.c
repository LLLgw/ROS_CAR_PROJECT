#include "drive_task.h"
#include "state.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_estop.h"

#define HALF_TRACK_M        0.286f						//半轮距
#define HALF_BASE_M         0.167f						//半轴距
#define TURN_RADIUS_M       (HALF_TRACK_M + HALF_BASE_M)//四驱差速底盘左右轮自转半径
#define ARC_TURN_RATIO      0.925f						//行进间转弯wz限制系数
#define ARC_VX_EPS_MPS      0.03f						//原地自转vx死区

#define WHEEL_DIAMETER_M    0.225f						//轮子直径
#define MOTOR_GEAR_RATIO    47.0f						//电机减速比
#define ENCODER_LINE        500.0f						//编码器线数
#define ENCODER_MULTI       4.0f						//四倍频计数

#define CONTROL_DT_S        0.01f						//积分时间间隔
#define PI_VALUE            3.1415926f					//圆周率

#define MAX_VX_MPS          0.45f						//vx最大速度
#define MAX_WZ_RADPS        1.20f						//wz最大速度
#define MAX_WHEEL_MPS       0.55f						//轮子最大速度

#define PWM_LIMIT           3500						//底盘PWM最大输出值
#define PWM_STATIC_OFFSET   480.0f						//静摩擦补偿
#define PWM_PER_MPS         5200.0f						//PWM与速度映射参数

#define SPEED_KP            2100.0f						//PID比例系数
#define SPEED_KI            800.0f						//PID积分系数
#define SPEED_I_LIMIT       900.0f						//PID积分项最大值

#define TARGET_STEP_MPS     0.0075f						//速度变化步长
#define SPEED_STOP_EPS      0.015f						//速度死区（速度上报防抖）

#define CHASSIS_STOP_V_EPS  0.025f						//底盘vx停稳判断
#define CHASSIS_STOP_W_EPS  0.05f						//底盘wz停稳判断

//接线方向修正
#define MOTOR_A_SIGN        1
#define MOTOR_B_SIGN        1
#define MOTOR_C_SIGN       -1
#define MOTOR_D_SIGN       -1
#define ENC_A_SIGN         -1
#define ENC_B_SIGN         -1
#define ENC_C_SIGN          1
#define ENC_D_SIGN          1

//轮子控制结构体
typedef struct
{
    float target_mps;
    float integral_pwm;
} WheelCtrl_t;

static WheelCtrl_t s_wheel[MOTOR_COUNT];		//轮子控制数组

//取绝对值工具函数
static float abs_float(float x)
{
    if (x < 0.0f)
        return -x;

    return x;
}

//范围限制工具函数
static float clamp_float(float x, float min, float max)
{
    if (x > max)
        return max;

    if (x < min)
        return min;

    return x;
}

//行进间转弯wz限制函数
static void limit_arc_turn(float vx, float *wz)
{
    float delta;
    float max_delta;
    float scale;

    if (abs_float(vx) < ARC_VX_EPS_MPS)
        return;

    delta = abs_float(*wz) * TURN_RADIUS_M;
    max_delta = abs_float(vx) * ARC_TURN_RATIO;

    if (delta <= max_delta)
        return;

    if (delta < 0.0001f)
        return;

    scale = max_delta / delta;
    *wz *= scale;
}

//平滑函数
static float approach_float(float now, float goal, float step)
{
    if (now < goal)
    {
        now += step;
        if (now > goal)
            now = goal;
    }
    else if (now > goal)
    {
        now -= step;
        if (now < goal)
            now = goal;
    }

    return now;
}

//轮速限制
static void scale_wheel_target(float target[MOTOR_COUNT])
{
    uint8_t i;
    float max_abs = 0.0f;
    float k;

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        if (abs_float(target[i]) > max_abs)
        {
            max_abs = abs_float(target[i]);
        }
    }

    if (max_abs <= MAX_WHEEL_MPS)
    {
        return;
    }

    k = MAX_WHEEL_MPS / max_abs;

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        target[i] *= k;
    }
}

//PWM限幅
static int16_t clamp_pwm(float value)
{
    if (value > PWM_LIMIT)
        value = PWM_LIMIT;

    if (value < -PWM_LIMIT)
        value = -PWM_LIMIT;

    return (int16_t)value;
}

//目标速度、积分项清零
static void clear_wheel_loop(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        s_wheel[i].target_mps = 0.0f;
        s_wheel[i].integral_pwm = 0.0f;
    }
}

//底盘停稳判断
static uint8_t chassis_is_settled(const float speed[MOTOR_COUNT])
{
    uint8_t i;

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        if (abs_float(s_wheel[i].target_mps) > SPEED_STOP_EPS)
        {
            return 0;
        }

        if (abs_float(speed[i]) > CHASSIS_STOP_V_EPS)
        {
            return 0;
        }
    }

    return 1;
}

//编码器计数转轮速
static float encoder_to_speed(int16_t delta)
{
    float count_per_rev;
    float wheel_len;

    count_per_rev = ENCODER_LINE * ENCODER_MULTI * MOTOR_GEAR_RATIO;	//500*4*47=9400次/圈
    wheel_len = PI_VALUE * WHEEL_DIAMETER_M;							//轮子周长

    return ((float)delta / count_per_rev) * wheel_len / CONTROL_DT_S;
}

//获取轮子速度
static void read_wheel_speed(float speed[MOTOR_COUNT])
{
    int16_t ca;
    int16_t cb;
    int16_t cc;
    int16_t cd;

    ca = Encoder_ReadDelta(ENCODER_A) * ENC_A_SIGN;
    cb = Encoder_ReadDelta(ENCODER_B) * ENC_B_SIGN;
    cc = Encoder_ReadDelta(ENCODER_C) * ENC_C_SIGN;
    cd = Encoder_ReadDelta(ENCODER_D) * ENC_D_SIGN;

    speed[MOTOR_A] = encoder_to_speed(ca);
    speed[MOTOR_B] = encoder_to_speed(cb);
    speed[MOTOR_C] = encoder_to_speed(cc);
    speed[MOTOR_D] = encoder_to_speed(cd);
}

//底盘速度反馈更新
static void update_chassis_feedback(const float speed[MOTOR_COUNT])
{
    float left_speed;
    float right_speed;

    left_speed = 0.5f * (speed[MOTOR_A] + speed[MOTOR_B]);
    right_speed = 0.5f * (speed[MOTOR_C] + speed[MOTOR_D]);

    g.feedback_speed.vx_mps = 0.5f * (left_speed + right_speed);
	g.feedback_speed.vy_mps = 0.0f;
	g.feedback_speed.wz_radps = (left_speed - right_speed) / (2.0f * TURN_RADIUS_M);

	if (abs_float(g.feedback_speed.vx_mps) < CHASSIS_STOP_V_EPS)
	{
		g.feedback_speed.vx_mps = 0.0f;
	}

	if (abs_float(g.feedback_speed.wz_radps) < CHASSIS_STOP_W_EPS)
	{
		g.feedback_speed.wz_radps = 0.0f;
	}
}

//PWM闭环输出
static int16_t speed_loop_output(uint8_t id, float target, float measured)
{
    float err;
    float abs_target;
    float out;
    float feed;

    abs_target = abs_float(target);

    if ((abs_target < SPEED_STOP_EPS) && (abs_float(measured) < SPEED_STOP_EPS))
    {
        s_wheel[id].integral_pwm = 0.0f;
        return 0;
    }

    err = target - measured;

    s_wheel[id].integral_pwm += err * SPEED_KI * CONTROL_DT_S;
    s_wheel[id].integral_pwm = clamp_float(s_wheel[id].integral_pwm,
                                           -SPEED_I_LIMIT,
                                           SPEED_I_LIMIT);

    feed = target * PWM_PER_MPS;									//前馈，减少P过程压力 

    if (abs_target >= SPEED_STOP_EPS)
    {
        if (target > 0.0f)
            feed += PWM_STATIC_OFFSET;
        else
            feed -= PWM_STATIC_OFFSET;
    }

    out = feed + SPEED_KP * err + s_wheel[id].integral_pwm;

    return clamp_pwm(out);
}

//停车清零
static void stop_drive(void)
{
    Motor_StopAll();
    clear_wheel_loop();

    g.motor_enable = 0;
    g.motion_mode = MOTION_MODE_STOP;
    g.control_source = CTRL_SOURCE_NONE;

    g.final_cmd.vx_mps = 0.0f;
    g.final_cmd.vy_mps = 0.0f;
    g.final_cmd.wz_radps = 0.0f;
    g.final_cmd.online = 0;

    g.feedback_speed.vx_mps = 0.0f;
    g.feedback_speed.vy_mps = 0.0f;
    g.feedback_speed.wz_radps = 0.0f;
}

//底盘驱动任务主函数
void Drive_TaskUpdate(void)
{
    float vx;
    float wz;
    float target[MOTOR_COUNT];
    float speed[MOTOR_COUNT];
    int16_t pwm[MOTOR_COUNT];
    uint32_t cmd_time;
    uint8_t i;

    read_wheel_speed(speed);				//获取轮子速度
    update_chassis_feedback(speed);			//底盘速度反馈
	
	g.emergency_stop = EStop_IsPressed();	//软件急停标志更新

    if (g.emergency_stop || g.low_battery)  //软件急停判断
    {
        stop_drive();
        return;
    }

    //接收目标vx和wz
	//select_source=ps2
	if ((g.selected_source == CTRL_SOURCE_PS2) && g.ps2_online && g.ps2_cmd.online)
    {
        vx = clamp_float(g.ps2_cmd.vx_mps, -MAX_VX_MPS, MAX_VX_MPS);
        wz = clamp_float(g.ps2_cmd.wz_radps, -MAX_WZ_RADPS, MAX_WZ_RADPS);
        cmd_time = g.ps2_cmd.last_update_ms;

        g.motion_mode = MOTION_MODE_PS2;
        g.control_source = CTRL_SOURCE_PS2;
    }
	//select_source=ROS
    else if ((g.selected_source == CTRL_SOURCE_ROS) && g.ros_online && g.ros_cmd.online)
    {
        vx = clamp_float(g.ros_cmd.vx_mps, -MAX_VX_MPS, MAX_VX_MPS);
        wz = clamp_float(g.ros_cmd.wz_radps, -MAX_WZ_RADPS, MAX_WZ_RADPS);
        cmd_time = g.ros_cmd.last_update_ms;

        g.motion_mode = MOTION_MODE_ROS;
        g.control_source = CTRL_SOURCE_ROS;
    }
	//select_source=none
	else
	{
		vx = 0.0f;
		wz = 0.0f;
		cmd_time = xTaskGetTickCount();

		g.motion_mode = MOTION_MODE_STOP;
		g.control_source = CTRL_SOURCE_NONE;
	}
	
	limit_arc_turn(vx, &wz);					//行进间转弯wz限制
	
	//逆运动学解算目标轮速
    target[MOTOR_A] = vx - wz * TURN_RADIUS_M;
    target[MOTOR_B] = vx - wz * TURN_RADIUS_M;
    target[MOTOR_C] = vx + wz * TURN_RADIUS_M;
    target[MOTOR_D] = vx + wz * TURN_RADIUS_M;
	
	scale_wheel_target(target);					//轮速限制

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        //平滑接近目标轮速
		s_wheel[i].target_mps = approach_float(s_wheel[i].target_mps,
                                               target[i],
                                               TARGET_STEP_MPS);
    }
	
	if ((vx == 0.0f) && (wz == 0.0f) && chassis_is_settled(speed))
	{
		Motor_StopAll();
		clear_wheel_loop();

		g.final_cmd.vx_mps = 0.0f;
		g.final_cmd.vy_mps = 0.0f;
		g.final_cmd.wz_radps = 0.0f;
		g.final_cmd.last_update_ms = cmd_time;
		g.final_cmd.online = 0;
		g.motor_enable = 0;
		return;
	}

    //PI过程
	pwm[MOTOR_A] = speed_loop_output(MOTOR_A, s_wheel[MOTOR_A].target_mps, speed[MOTOR_A]);
    pwm[MOTOR_B] = speed_loop_output(MOTOR_B, s_wheel[MOTOR_B].target_mps, speed[MOTOR_B]);
    pwm[MOTOR_C] = speed_loop_output(MOTOR_C, s_wheel[MOTOR_C].target_mps, speed[MOTOR_C]);
    pwm[MOTOR_D] = speed_loop_output(MOTOR_D, s_wheel[MOTOR_D].target_mps, speed[MOTOR_D]);

    Motor_SetPwm(MOTOR_A, MOTOR_A_SIGN * pwm[MOTOR_A]);
    Motor_SetPwm(MOTOR_B, MOTOR_B_SIGN * pwm[MOTOR_B]);
    Motor_SetPwm(MOTOR_C, MOTOR_C_SIGN * pwm[MOTOR_C]);
    Motor_SetPwm(MOTOR_D, MOTOR_D_SIGN * pwm[MOTOR_D]);

    g.final_cmd.vx_mps = vx;
    g.final_cmd.vy_mps = 0.0f;
    g.final_cmd.wz_radps = wz;
    g.final_cmd.last_update_ms = cmd_time;
    g.final_cmd.online = 1;

    g.motor_enable = 1;
}
