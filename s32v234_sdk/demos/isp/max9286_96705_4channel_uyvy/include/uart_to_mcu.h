#ifndef __UART_TO_MCU_H__
#define __UART_TO_MCU_H__

#include <semaphore.h>
#include <pthread.h>
#include <sys/time.h>
#include "unistd.h"

#define STAMPNUMS 10 //20
#define BACKNUMS  9  //9--20ms//18---10ms

typedef struct {
	unsigned char 	f_start;
	unsigned char	  id_flag;
	unsigned char  	d_len;
	unsigned char	  *data;
	unsigned char	  xor_flag;
}mcu_soc_frame_t;

typedef struct PcReadData
{
	char rader1_alarm_level;			//超声波1号的报警等级
	char rader2_alarm_level;			//超声波2号的报警等级
	char rader3_alarm_level;			//超声波3号的报警等级
	char rader4_alarm_level;			//超声波4号的报警等级
	char rader5_alarm_level;			//超声波5号的报警等级
	char rader6_alarm_level;			//超声波6号的报警等级
	char rader7_alarm_level;			//超声波7号的报警等级
	char rader8_alarm_level;			//超声波8号的报警等级
	char rader9_alarm_level;			//超声波9号的报警等级
	char rader10_alarm_level;			//超声波10号的报警等级
	char rader11_alarm_level;			//超声波11号的报警等级
	char rader12_alarm_level;			//超声波12号的报警等级
	char rader13_alarm_level;			//超声波13号的报警等级
	char rader14_alarm_level;			//超声波14号的报警等级
	char rader15_alarm_level;			//超声波15号的报警等级
	char rader16_alarm_level;			//超声波16号的报警等级

	short int parking_rect_point0_x;	//显示车位角点坐标X1
	short int parking_rect_point0_y;	//显示车位角点坐标Y1
	short int parking_rect_point1_x;	//显示车位角点坐标X2
	short int parking_rect_point1_y;	//显示车位角点坐标Y2
	short int parking_rect_point2_x;	//显示车位角点坐标X3
	short int parking_rect_point2_y;	//显示车位角点坐标Y3
	short int parking_rect_point3_x;	//显示车位角点坐标X4
	short int parking_rect_point3_y;	//显示车位角点坐标Y4

	char car_paring_status;				//泊车状态
	short int TimeStampex[3]; 			//TimeStampex[0]：time stampex   TimeStampex[1]：time stampey  TimeStampex[2]：time stampey
	unsigned int TimeStampex_Couter;
	unsigned int Timer_Couter;          //时间计数/对钟
	
	short int trail_arc_radius ;		//泊车轨迹半径
	short int trail_arc_angle;			//泊车轨迹弧度
	char  switchCannelnum;		//泊车模式选择显示01//key1
	char gear_status_actual;			//实际档位信息
	char gear_status_pre;				//期望档位信息
	char McuKey;						//转向灯
	short int WheelSpeed[5];  			//WheelSpeed[0]:左前轮轮速WheelSpeed[1]:右前轮轮速WheelSpeed[2]:左后轮轮速
										//WheelSpeed[3]:右后轮轮速WheelSpeed[4]:前轮转�

	short int CarSpeed;
	char Steering_Wheel_angle;
	short int yaw_rate;						//摆角,单位 度/S  ,真实值=原始值/100
	short int acceleration_x;				//x轴加速度，单位m/s2,真实值=原始值/100
	short int acceleration_y;				//Y轴加速度，单位m/s2,真实值=原始值/100
	short int actual_steering_wheel_angle;	//方向盘转角信息，单位：度，左为正，右为负，真实值=原始值/100
}PCREADDATA;

typedef struct PcWriteData
{
	unsigned char Pc_Write_Flag ;	//PC写数据标志     
	/**		   bit7.6.5.4.3.2.1.0					
				  ｜｜｜｜｜｜｜｜--- 第一组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜｜｜—---- 第二组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜｜—----—— 第三组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜｜—------—— 第四组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜｜—--------—— 第五组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜｜—----------—— 第六组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜｜—------------—— 第七组数据发送标志位 1：执行写数据  0：不执行写数据
				  ｜—--------------—— 第八组数据发送标志位 1：执行写数据  0：不执行写数据
	**********************************************************/
	short int PC_CarPark_P0Point[2];  //P0点坐标（x,y）x1:PC_CarPark_P0Point[0]  y1:PC_CarPark_P0Point[1]
	short int PC_CarPark_P1Point[2];  //P1点坐标（x,y）x2:PC_CarPark_P1Point[0]  y2:PC_CarPark_P1Point[1]
	short int PC_CarPark_P2Point[2];  //P2点坐标（x,y）x3:PC_CarPark_P2Point[0]  y3:PC_CarPark_P2Point[1]
	short int PC_CarPark_P3Point[2];  //P3点坐标（x,y）x4:PC_CarPark_P3Point[0]  y4:PC_CarPark_P3Point[1]
	char  PC_ZhiXinDu;      //置信度
	short int PC_TimeStampeX;//车位识别TimeStampeX
	short int PC_TimeStampeY;//车位识别TimeStampeY
	short int PC_TimeStampeZ;//车位识别TimeStampeZ
	short int PC_CarFront_Ear_X;//车位前侧耳朵外侧点坐标值X
	short int PC_CarFront_Ear_Y;//车位前侧耳朵外侧点坐标值Y
//	short int PC_Front_Ear_TimeStampeX;//前耳朵识别Time stampeX
//	short int PC_Front_Ear_TimeStampeY;//前耳朵识别Time stampeY
	short int PC_CarBack_Ear_X;//车位后侧耳朵外侧点坐标值X
	short int PC_CarBack_Ear_Y;//车位后侧耳朵外侧点坐标值y
//	short int PC_Back_Ear_TimeStampeX;//后耳朵识别Time stampeX
//	short int PC_Back_Ear_TimeStampeY;//后耳朵识别Time stampeY
	short int PC_FrontNear_TyreGound_X;//前侧近端轮胎接地点坐标值X
	short int PC_FrontNear_TyreGound_Y;//前侧近端轮胎接地点坐标值Y
	short int PC_FrontFar_TyreGound_X;//前侧远端轮胎接地点坐标值X
	short int PC_FrontFar_TyreGound_Y;//前侧远端轮胎接地点坐标值Y
//	short int PC_Front_TyreGound_TimeStampeX;//前侧轮胎接地点识别Time stampeX
//	short int PC_Front_TyreGound_TimeStampeY;//前侧轮胎接地点识别Time stampeX

	short int PC_BackNear_TyreGound_X;//后侧近端轮胎接地点坐标值X
	short int PC_BackNear_TyreGound_Y;//后侧近端轮胎接地点坐标值Y
	short int PC_BackFar_TyreGound_X;//后侧远端轮胎接地点坐标值X
	short int PC_BackFar_TyreGound_Y;//后侧远端轮胎接地点坐标值Y
//	short int PC_Back_TyreGound_TimeStampeX;//后侧轮胎接地点识别Time stampeX
//	short int PC_Back_TyreGound_TimeStampeY;//后侧轮胎接地点识别Time stampeX

	short int PC_Front_Bumper_X1;//车位前侧保险杠区域坐标X1
	short int PC_Front_Bumper_Y1;//车位前侧保险杠区域坐标Y1
	short int PC_Front_Bumper_X2;//车位前侧保险杠区域坐标X2
	short int PC_Front_Bumper_Y2;//车位前侧保险杠区域坐标Y2
	short int PC_Front_Bumper_X3;//车位前侧保险杠区域坐标X3
	short int PC_Front_Bumper_Y3;//车位前侧保险杠区域坐标Y3
	short int PC_Front_Bumper_X4;//车位前侧保险杠区域坐标X4
	short int PC_Front_Bumper_Y4;//车位前侧保险杠区域坐标Y4

	short int PC_Front_Bumper_TimeStampeX;//前侧保险杠识别Time stampeX
	short int PC_Front_Bumper_TimeStampeY;//前侧保险杠识别Time stampeY
	short int PC_Front_Bumper_TimeStampeZ;//前侧保险杠识别Time stampeY

	short int PC_Back_Bumper_X1;//车位后侧保险杠区域坐标X1
	short int PC_Back_Bumper_Y1;//车位后侧保险杠区域坐标Y1
	short int PC_Back_Bumper_X2;//车位后侧保险杠区域坐标X2
	short int PC_Back_Bumper_Y2;//车位后侧保险杠区域坐标Y2
	short int PC_Back_Bumper_X3;//车位后侧保险杠区域坐标X3
	short int PC_Back_Bumper_Y3;//车位后侧保险杠区域坐标Y3
	short int PC_Back_Bumper_X4;//车位后侧保险杠区域坐标X4
	short int PC_Back_Bumper_Y4;//车位后侧保险杠区域坐标Y4

	short int PC_Back_Bumper_TimeStampeX;//后侧保险杠识别Time stampeX
	short int PC_Back_Bumper_TimeStampeY;//后侧保险杠识别Time stampeY
	short int PC_Back_Bumper_TimeStampeZ;//后侧保险杠识别Time stampeY
}PCWRITEDATA;
typedef struct targetPlaceData
{
	short int Dnn_Scan_Ok_Flag;
	short int Customer_Select_Ok_Flag;

	short int Target_CarPark_P0Point[2];  //P0点坐标（x,y）x1:PC_CarPark_P0Point[0]  y1:PC_CarPark_P0Point[1]
	short int Target_CarPark_P1Point[2];  //P1点坐标（x,y）x2:PC_CarPark_P1Point[0]  y2:PC_CarPark_P1Point[1]
	short int Target_CarPark_P2Point[2];  //P2点坐标（x,y）x3:PC_CarPark_P2Point[0]  y3:PC_CarPark_P2Point[1]
	short int Target_CarPark_P3Point[2];  //P3点坐标（x,y）x4:PC_CarPark_P3Point[0]  y4:PC_CarPark_P3Point[1]
	int   TargetConfidence;
    int   Target_TimeStamp;
    short int   Targetx;
    short int   Targety;
    short int   Targetz;
}TARGETPLACEDATA;

PCREADDATA 	* get_data_from_mcu(void);
void write_data_into_mcu(PCWRITEDATA *p,unsigned char Write_Flag);


void write_data(void);

#define F_START_MCU			    0x10
#define F_START_SOC			    0x01

#define ID_HANDSSHAKE_MCU	    0x10
#define ID_HANDSSHAKE_SOC	    0x01
#define ID_HEARTBEAT_MCU        0x20
#define ID_HEARTBEAT_SOC        0x02
#define ID_ULTRASONIC_ALARM	    0x22
//#define ID_PARKING_DISTANCE     0x26
#define ID_CAR_STATUS		    0x28
#define ID_TRAIL_LINI1          0x2A
#define ID_TRAIL_ARC      	    0x2B
#define ID_TRAIL_LINI2      	0x2C
#define ID_KEY_MCU        		0xC2	
#define ID_KEY_SOC        		0x30
#define ID_KEY_SOC        		0x30
#define ID_GEAR_MCU				0x31
#define ID_PARK_RECT_POINT1		0x26
#define ID_PARK_RECT_POINT2		0x27

//below is added by che
#define ID_WHEELSPEED           0X52
#define ID_TIMESTAMPEX          0X50  //TimeStampex
extern pthread_mutex_t  mcu_data_mutex; 
extern PCREADDATA 		McuSend_PcReadData,McuSend_PcReadData_Pc;
extern PCWRITEDATA 	McuReceive_PcWriteData;
//up is added by che

/*********************************for IPC ***********************************/
typedef struct
{
    int S32v_TimeStamp;
}BasicData;

typedef struct
{
    unsigned char Mem_FrontCam[1280*720*2];
	unsigned char Mem_BackCam[1280*720*2];
	unsigned char Mem_LeftCam[1280*720*2];
	unsigned char Mem_RightCam[1280*720*2];
	int TimeStamp;
	int x;
	int y;
	int z;
}CamData;

typedef struct
{
   float ParkPlace[10*4*2];
   int   ParkPlaceMode[10];
   int   ParkPlaceNum;
   int   TimeStamp;
   int   x;
   int   y;
   int   z;
   int   ParkConfidence[10];
   float TargetPlace[4*2];
   int   Target_TimeStamp;
   int   Targetx;
   int   Targety;
   int   Targetz;
   int   TargetConfidence;
}ParkingPlace;

typedef struct
{
   	char car_paring_status;				//泊车状态
	short int TimeStampex[3]; 			//TimeStampex[0]：time stampex   TimeStampex[1]：time stampey  TimeStampex[2]：time stampey
	unsigned int TimeStampex_Couter;
	
	short int trail_arc_radius ;		//泊车轨迹半径
	short int trail_arc_angle;			//泊车轨迹弧度
	short int WheelSpeed[5];  			//WheelSpeed[0]:左前轮轮速WheelSpeed[1]:右前轮轮速WheelSpeed[2]:左后轮轮速										//WheelSpeed[3]:右后轮轮速WheelSpeed[4]:前轮转�
	short int CarSpeed;					//车速，单位：km/h,,真实值=原始值/100
	short int yaw_rate;					//摆角,单位 度/S  ,真实值=原始值/100
	short int acceleration_x;			//x轴加速度，单位m/s2,真实值=原始值/100
	short int acceleration_y;			//Y轴加速度，单位m/s2,真实值=原始值/100
	short int actual_steering_wheel_angle;		//方向盘转角信息，单位：度，左为正，右为负，真实值=原始值/10
}McuSendDada;

typedef struct
{		
	unsigned int TimeStampex_Couter;
	short int TimeStampex[3]; 			//TimeStampex[0]：time stampex   TimeStampex[1]：time stampey  TimeStampex[2]：time stampez
	short int lane_width;
	float left_a0;
	float left_a1;
	float left_a2;
	float left_a3;
	short int left_width;
	char left_type;
	short int left_length;
	char left_confidence;
	float right_a0;
	float right_a1;
	float right_a2;
	float right_a3;
	short int right_width;
	char right_type;
	short int right_length;
	char right_confidence;
}LanelineData;
typedef struct
{
	unsigned int TimeStampex_Couter;
	short int TimeStampex[3]; 			//TimeStampex[0]：time stampex   TimeStampex[1]：time stampey  TimeStampex[2]：time stampez
	char num;
	struct objectinfo
	{
		short int objectid;
		char  objecttype;
		char  confidence;
		short int P1_x;
		short int P1_y;
		short int P2_x;
		short int P2_y;
		short int P3_x;
		short int P3_y;
		short int P4_x;
		short int P4_y;
		short int P1_real_x;
		short int P1_real_y;
		short int P2_real_x;
		short int P2_real_y;
		short int height;
		char status;
		float velocity;
		short int direction;
		char validity;
	}objarray[16];
}ObstacleData;

typedef struct
{
	int IP_TotalOdometer;//累计里程
	char IP_OdometerSts;//累计里程有效标志位
	char IVI_LanguageSetSts;//语言设置状态
	char BCM_PwrDistributionSts;//电源模式
	char BCM_RKETrunkUnlockCommand;//RKE解尾门命令
	char BCM_RKEUnlockCommand;//RKE解锁信号
	char BCM_RKELockCommand;//RKE闭锁信号

	char BCM_DoorAjarFrntLeSts;//左前门开关状态
	char BCM_DoorAjarFrntRiSts;//右前门开关状态
	char BCM_DoorAjarReLeSts;//左后门开关状态
	char BCM_DoorAjarReRiSts;//右后门开关状态
	char BCM_HoodAjarSts;//前舱盖开关状态
	char BCM_TrAjarSts;//尾门开关状态
	char BCM_ChargingCoverSts;//充电口盖状态
	char BCM_LeTurnIndcrLiSts;//左转向灯状态
	char BCM_RiTurnIndcrLiSts;//右转向灯状态
	char BCM_TurnSigSw;//转向开关信号状态
	char BCM_emergrncyLampsts;//双闪灯状态
	char BCM_HignBeamSwitch;//远光灯开关输入
	char BCM_MirrorsSts;//后视镜折叠状态
	char BCM_HiBeamSts;//远光灯状态
	char BCM_LoBeamSts;//近光灯状态
	char BCM_BrkLiSts;//刹车灯状态
	char BCM_AutomaticLampSts;//自动灯光状态
	char CS_DATA_CS_version;//CS版本号
	char CS_DATA_CRC;//CS数据CRC

	char IVI_APA_HVPSele;//水平垂直泊入功能选择
	char IVI_RadarSoundSet;//雷达声音开启_设置
	char IVI_AVMReq;//全景状态切换请求
	char IVI_Touch_Active;//触摸屏触摸状态
	char IVI_AVM_InterfaceOpenReq;//AVM界面打开退出请求
	short int IVI_Touch_X_Coor;//触摸点x坐标
	short int IVI_Touch_Y_Coor;//触摸点y坐标
	short int CCM_OutsideTemp;//车外温度
		
	char GW_VIN_Value_1;//VIN码:用于传输VIN码数据 7N+1，GW_3AA
	char GW_VIN_Value_2;//VIN码:用于传输VIN码数据 7N+2，GW_3AA
	char GW_VIN_Value_3;//VIN码:用于传输VIN码数据 7N+3，GW_3AA
	char GW_VIN_Value_4;//VIN码:用于传输VIN码数据 7N+4，GW_3AA
	char GW_VIN_Value_5;//VIN码:用于传输VIN码数据 7N+5，GW_3AA
	char GW_VIN_Value_6;//VIN码:用于传输VIN码数据 7N+6，GW_3AA
	char GW_VIN_Value_7;//VIN码:用于传输VIN码数据 7N+7，GW_3AA
	char GW_VINFrameNumber;//VIN帧序号:用于标记发送报文帧序号N，GW_3AA
}IVISendData;

typedef struct  
{
	unsigned int APP_RPA_Termination :1;
	unsigned int APP_RPA_MatrixConfirmSts :1;
	unsigned int APP_BCM_FlameoutReqSts :2;
	unsigned int APP_RPA_POC_ActAndVehOn :3;
	unsigned int Phone_Bluetooth_CntSts :1;

	unsigned int  :2;
	unsigned int APP_RPA_InterfaceSts :3;
	unsigned int APP_RPA_SwSts :3;

	unsigned int APP_RPA_RemPocDir :3;
	unsigned int APP_RPA_ParkingPathReq :4;
	unsigned int  :1;

	unsigned int  :1;
	unsigned int TBOX_RPA_ReqSts :3;
	unsigned int TBOX_RPA_InhibitCode :4;
}TBOX_15E_t; 

typedef struct  
{
	unsigned int IVI_RPA_CustomPositionType :2;
	unsigned int IVI_RPA_CustomButtonState :2;
	unsigned int IVI_RPA_RemButtonTouchState :2;
	unsigned int IVI_APA_HVPSele_1 :2;

	unsigned int IVI_RPA_CustomPositionConfirm :2;
	unsigned int IVI_RPA_PocDir :3;
	unsigned int IVI_RPA_Custom_Position_X :11;

	unsigned int IVI_RPA_PathDelect :4;
	unsigned int IVI_RPA_ParkingPathReq :4;

	unsigned int IVI_RPA_SideRadarSoundSet :2;
	unsigned int IVI_RPA_SpeedOrPerformanceSet :2;
	unsigned int IVI_RPA_ParkslotDir :2;
	unsigned int IVI_RPA_MemoryParking :2;

	unsigned int IVI_RPA_Custom_Position_Y :11;
	unsigned int  :1;
	unsigned int IVI_RollingCounter_2FE :4;
	unsigned int IVI_Checksum_2FE :8;
}IVI_2FE_t;

typedef struct
{
	char AVM_Err_Sta;//全景错误状态
	char AVM_Type;//全景类别
	char AVM_APAModeViewLayout;//AVM-APA模式视图布局
	char AVM_ViewDisplayOrientation;//AVM视图显示方位
	char AVM_ViewReq;//AVM向IVI输出画面请求
}SoctoIVIData;

typedef struct
{
	unsigned int RPA_MemoryWorkingSts :1;
	unsigned int RPA_RemWorkingSts_Vd :1;
	unsigned int RPA_RemWorking_Sts :1;
	unsigned int RPA_TBOX_Sts :2;
	unsigned int RPA_APP_RemButtonSts :3;

	unsigned int RPA_ESP_AbnormalRqst :2;
	unsigned int RPA_Poc_Dir :3;
	unsigned int RPA_BCM_HornSts :1;
	unsigned int RPA_BCM_EmgLightSts :1;
	unsigned int RPA_MemoryWorkingSts_Vd :1;

	// byte3和byte4需转换
	unsigned int PAS_APA_Indication_1 :3;
	unsigned int PAS_APA_Quit_Indication_1 :4;
	unsigned int PAS_APA_Recover_INT_Indication_1 :3;
	unsigned int RPA_ParkingRoute_1 :2;
	unsigned int RPA_ParkingRoute_2 :2;
	unsigned int RPA_ParkingRoute_3 :2;

	// 可不转换
	unsigned int RPA_ParkingRoute_4 :2;
	unsigned int PAS_APA_DrRqst_1 :6;

	unsigned int RPA_CallingRoute_3 :2;
	unsigned int RPA_CallingRoute_2 :2;
	unsigned int RPA_CallingRoute_1 :2;
	unsigned int RPA_ParkingRoute_5 :2;

	unsigned int RPA_RollingCounter_178 :4;
	unsigned int RPA_CallingRoute_5 :2;
	unsigned int RPA_CallingRoute_4 :2;

	unsigned int RPA_Checksum_178 :8;
}RPA_178_t;

typedef struct
{
	long RPA_IVI_SideRadarSoundSetResp :2;
	long RPA_IVI_PerformanceSetResp :2;
	long RPA_IVI_CustomButtonAva :2;
	long RPA_IVI_RemButtonAva :2;

	long RPA_IVI_CustomPraPositionSts :3;
	long RPA_IVI_CustomVerPositionSts :3;
	long RPA_IVI_CustomPosition_X :11;
	long RPA_IVI_CustomPosition_Y :11;
	long RPA_PDC_WarningLocation :2;
	long RPA_PDC_DISTANCE_FSL :2;

	// 可不转换  
	long  :2;
	long RPA_PDC_DISTANCE_RSR :2;
	long RPA_PDC_DISTANCE_RSL :2;
	long RPA_PDC_DISTANCE_FSR :2;

	long RPA_RollingCounter_179 :4;
	long  :4;
	long RPA_Checksum_179 :8;
}RPA_179_t;

/*****************************************************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif
 void *Uart_meg_thread(void *t);
 void *Uart_TX_thread(void *t) ;
 int uart_thread_create(void);
 int AIWAYS_get_coordinate_area(short int x, short int y);

#ifdef __cplusplus
}
#endif



#endif
