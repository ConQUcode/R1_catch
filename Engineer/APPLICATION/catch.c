#include "catch.h"
#include "daemon.h"
#include "remote.h"
#include "feite_motor.h"
#include "bsp_dwt.h"
#include "DJI_motor.h"
#include "tim.h"
#include "cmsis_os.h"

FTMotor_instance *FT_1,*FT_2,*FT_3,*FT_4;
static DJIMotor_Instance *DJM2006,*DJM3508;
RC_ctrl_t *rc_cmd;
int IR_sensor_level;
float control;
int a=0;
static int8_t is_init_2006 = 0;
static int8_t is_init_3508 = 0;


void duoji_init(){

	FTMotor_Init_Config_s ftmotor_config ={
		.usart_init_config = {
			.usart_handle = &huart1,
		},
		.motor_set = {
			.ID = 1,
			.MemAddr = SMS_STS_ACC,
      .Fun = INST_WRITE,
		},
		.motor_ref = {
			.Position =  0,
			.Speed =  2250,
			.ACC = 50,
		},
	};
	FT_1 = FTMotorInit(&ftmotor_config);
  
	ftmotor_config.motor_set.ID = 2;
  FT_2 = FTMotorInit(&ftmotor_config);

	ftmotor_config.motor_set.ID = 3;
  FT_3 = FTMotorInit(&ftmotor_config);

	ftmotor_config.motor_set.ID = 4;
  FT_4 = FTMotorInit(&ftmotor_config);

}

void dianji_init(){

			Motor_Init_Config_s dianji_config = {
			.can_init_config = {
					.can_handle = &hcan1,
					.tx_id      = 1,
			},
			.controller_param_init_config = {
				.angle_PID = {
                .Kp                = 3,
                .Ki                = 1.5,
                .Kd                = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
                .IntegralLimit     = 60000,
                .MaxOut            = 40000,
                .Derivative_LPF_RC = 0.01,
            },
					.speed_PID = {
							.Kp = 3,  // 7
							.Ki = 0.005,// 0.01
							.Kd = 0.004,//0.008
							// .CoefA         = 0.2,
							// .CoefB         = 0.3,
							.Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
							.IntegralLimit = 10000,
							.MaxOut        = 15000,
					},
				
					.current_PID = {
							.Kp            = 0.2, // 0.4
							.Ki            = 0.0006, // 0.001
							.Kd            = 0,
							.Improve       = PID_Integral_Limit,
							.IntegralLimit = 10000,
							.MaxOut        = 15000,
							// .DeadBand      = 0.1,
					},
			},
			.controller_setting_init_config = {
					.speed_feedback_source = MOTOR_FEED,
					.outer_loop_type       = ANGLE_LOOP, 
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP   ,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
			},
			.motor_type = M2006
			};
      DJM2006 = DJIMotorInit(&dianji_config);
	
			
//M3508
			Motor_Init_Config_s M3508_config = {
    .can_init_config = {
        .can_handle = &hcan1,
        .tx_id      = 2,
    },
    .controller_param_init_config = {
				.angle_PID = {
            .Kp                = 10	,
		  			.Ki                = 0.5,
            .Kd                = 0.1,
            .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
            .IntegralLimit     = 50000,
            .MaxOut            = 50000,
            .Derivative_LPF_RC = 0.01,
            },
        .speed_PID = {
            .Kp = 4,
            .Ki = 0.025,
            .Kd = 0.02,
            .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 10000,
            .MaxOut        = 15000,
        },
        .current_PID = {
            .Kp            = 0.5,
            .Ki            = 0.01,
            .Kd            = 0,
            .Improve       = PID_Integral_Limit,
            .IntegralLimit = 10000,
            .MaxOut        = 50000,
        },
    },
    .controller_setting_init_config = {
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type       = ANGLE_LOOP, 
        .close_loop_type       = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
        .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
        .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
    },
    .motor_type = M3508
};
			DJM3508 = DJIMotorInit(&M3508_config);
}


static void LiftInit() {
    // 状态 A：初始化 M2006 (IR 传感器找零)
    if (!is_init_2006) {
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 1) {
            DJIMotorEnable(DJM2006);
            DJIMotorOuterLoop(DJM2006, SPEED_LOOP);
            DJIMotorSetRef(DJM2006, -4000); // 速度不要太快
        } else {
            osDelay(10); // 消抖
            if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 0) {
                DJIMotorStop(DJM2006);
                DJIMotorReset(DJM2006);
                DJIMotorOuterLoop(DJM2006, ANGLE_LOOP);
                DJIMotorSetRef(DJM2006, 0);
                is_init_2006 = 1;
            }
        }
        return; // M2006 没搞定前不跑下面的
    }

    // 状态 B：初始化 M3508 (电流堵转找零)
    if (!is_init_3508) {
        static uint16_t stall_cnt = 0;
        
        // 切换到速度环去撞限位
        DJIMotorEnable(DJM3508);
        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
        DJIMotorSetRef(DJM3508, -3000); // 用较小的速度去撞

        // 连续多次采样电流，防止瞬时噪声误判
        // M3508 堵转电流通常在 8000~10000 以上，4500 略低
        if (abs(DJM3508->measure.real_current) > 4200) {
              osDelay(5);
					if (abs(DJM3508->measure.real_current) > 4200) {
            DJIMotorStop(DJM3508);
            DJIMotorReset(DJM3508);
            DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
            DJIMotorSetRef(DJM3508, 0); // 锁定在零位
            is_init_3508 = 1;
					}
        }
    }
}

void catch_init(){
	DWT_Init(168);
	rc_cmd = RemoteControlInit(&huart3);
  duoji_init();
  dianji_init();
}

//爪子闭合
void feite_catch(){

  FT_1 ->motor_ref.Position = 350;
	FT_2 ->motor_ref.Position = 350;
  FT_3 ->motor_ref.Position = 350;
	FTMotorControl();

}
//爪子张开(小角度)
void feite_putdown(){
  FT_1 ->motor_ref.Position = 250;
	FT_2 ->motor_ref.Position = 650;
  FT_3 ->motor_ref.Position = 500;
	FTMotorControl();

}

void feite_open(){

  FT_1 ->motor_ref.Position = 250;
	FT_2 ->motor_ref.Position = 650;
  FT_3 ->motor_ref.Position = 500;
	FTMotorControl();

}

//整个抓取动作
void catchpole(){
	DJIMotorEnable(DJM3508);
  DJIMotorSetRef (DJM3508,5000);
	osDelay(1000);
	//伸出
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
	DJIMotorEnable(DJM2006);
	//把爪子转动到平面
  DJIMotorSetRef(DJM2006,5000);
	feite_catch();
	DJIMotorSetRef (DJM3508,7000);
	
}


//调整使得矛杆以地面为基准
void adapt(){
	
  DJIMotorSetRef(DJM3508,5000);
  osDelay(500);
	feite_putdown();
	osDelay(200);
	feite_catch();

}

void putdown(){
	
DJIMotorSetRef(DJM2006,0);
osDelay(200);
	//缩回
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
feite_open();

}


void catch_all(){
	DJIMotorSetRef(DJM3508,15000);
  IR_sensor_level=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
	LiftInit();
////	DJIMotorEnable(DJM3508);
////  DJIMotorSetRef (DJM3508,18000);
//	DJIMotorOuterLoop(DJM3508,SPEED_LOOP);
//  DJIMotorSetRef(DJM3508,1000);
//	if(IR_sensor_level==0){
//	DJIMotorReset(DJM3508);
//	DJIMotorStop (DJM3508);
//	DJIMotorEnable(DJM3508);
//		int totalangle=DJM3508->measure.total_angle;
////	DJIMotorOuterLoop(DJM3508,ANGLE_LOOP);
////	DJIMotorSetRef(DJM3508,0);
//	
//	};
//DJIMotorSetRef(DJM2006,9250);
}
	
