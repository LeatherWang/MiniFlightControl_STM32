/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_IMU.c
 * 描述    ：姿态解算函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "ANO_IMU.h"
#include "mymath.h"
#include "filter.h"

_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};

_lf_t err_lf_x;
_lf_t err_lf_y;
_lf_t err_lf_z;

_xyz_f_st vec_err_i;

void IMU_update(float dT,_xyz_f_st *gyr, _xyz_f_st *acc,_imu_st *imu)
{
	float kp = 10.0f,ki = 0.008f;
	
	float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;//q0q0,
	float w_q,x_q,y_q,z_q;
	float acc_length,q_length;
	_xyz_f_st acc_norm;
	_xyz_f_st vec_err;
	_xyz_f_st d_angle;
	  

    //
    w_q = imu->w;
    x_q = imu->x;
    y_q = imu->y;
    z_q = imu->z;
	
//		q0q0 = w_q * w_q;							
		q0q1 = w_q * x_q;
		q0q2 = w_q * y_q;
		q1q1 = x_q * x_q;
		q1q3 = x_q * z_q;
		q2q2 = y_q * y_q;
		q2q3 = y_q * z_q;
		q3q3 = z_q * z_q;
		q1q2 = x_q * y_q;
		q0q3 = w_q * z_q;
	
    //
		
    // 加速度计的读数，单位化。
    acc_length = my_sqrt(my_pow(acc->x) + my_pow(acc->y) + my_pow(acc->z));
    acc_norm.x = acc->x / acc_length;
    acc_norm.y = acc->y / acc_length;
    acc_norm.z = acc->z / acc_length;
    //
		
	// 载体坐标下的x方向向量，单位化。
    imu->x_vec.x = 1 - (2*q2q2 + 2*q3q3);
    imu->x_vec.y = 2*q1q2 - 2*q0q3;
    imu->x_vec.z = 2*q1q3 + 2*q0q2;
		
	// 载体坐标下的y方向向量，单位化。
    imu->y_vec.x = 2*q1q2 + 2*q0q3;
    imu->y_vec.y = 1 - (2*q1q1 + 2*q3q3);
    imu->y_vec.z = 2*q2q3 - 2*q0q1;
		
    // 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。
    imu->z_vec.x = 2*q1q3 - 2*q0q2;
    imu->z_vec.y = 2*q2q3 + 2*q0q1;
    imu->z_vec.z = 1 - (2*q1q1 + 2*q2q2);
		
	// 计算载体坐标下的运动加速度。(与姿态解算无关)
	imu->a_acc.x = acc->x - 9800 *imu->z_vec.x;
	imu->a_acc.y = acc->y - 9800 *imu->z_vec.y;
	imu->a_acc.z = acc->z - 9800 *imu->z_vec.z;
    
    // 测量值与等效重力向量的叉积（计算向量误差）。
    vec_err.x =  (acc_norm.y * imu->z_vec.z - imu->z_vec.y * acc_norm.z);
    vec_err.y = -(acc_norm.x * imu->z_vec.z - imu->z_vec.x * acc_norm.z);
    vec_err.z = 0;// -(acc_norm.y * imu->z_vec.x - imu->z_vec.y * acc_norm.x);
		
	//截止频率1hz的低通限幅滤波
	limit_filter(dT,0.2f,&err_lf_x,vec_err.x);
	limit_filter(dT,0.2f,&err_lf_y,vec_err.y);
	limit_filter(dT,0.2f,&err_lf_z,vec_err.z);
	
	//误差积分
	vec_err_i.x += err_lf_x.out *dT *ki;
	vec_err_i.y += err_lf_y.out *dT *ki;
	vec_err_i.z += err_lf_z.out *dT *ki;
		
    // 构造增量旋转（含融合纠正）。
    d_angle.x = (gyr->x *RAD_PER_DEG + (err_lf_x.out + vec_err_i.x) * kp) * dT / 2 ;
    d_angle.y = (gyr->y *RAD_PER_DEG + (err_lf_y.out + vec_err_i.y) * kp) * dT / 2 ;
    d_angle.z = (gyr->z *RAD_PER_DEG + (err_lf_z.out + vec_err_i.z) * kp) * dT / 2 ;
    
    // 计算姿态。
    imu->w = w_q           - x_q*d_angle.x - y_q*d_angle.y - z_q*d_angle.z;
    imu->x = w_q*d_angle.x + x_q           + y_q*d_angle.z - z_q*d_angle.y;
    imu->y = w_q*d_angle.y - x_q*d_angle.z + y_q           + z_q*d_angle.x;
    imu->z = w_q*d_angle.z + x_q*d_angle.y - y_q*d_angle.x + z_q;
		
	q_length = my_sqrt(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w /= q_length;
    imu->x /= q_length;
    imu->y /= q_length;
    imu->z /= q_length;
	
		imu->pit = asin(2*q1q3 - 2*q0q2)*57.30f;
		imu->rol = fast_atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f; 
		imu->yaw = -fast_atan2(2*q1q2 + 2*q0q3, -2*q2q2-2*q3q3 + 1)*57.30f; 
  
    
}



#define Kp 10.0f                       // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.008f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.003f                   // half the sample period2¨¦?¨´?¨¹?¨²¦Ì?¨°?¡ã?
float roll_radian,pitch_radian;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{      
  float norm;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // ?¨¨¡ã??aD?¨®?¦Ì?¦Ì?¦Ì??¦Ì??o?
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
  if(ax*ay*az==0)
     return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc¨ºy?Y1¨¦¨°??¡¥
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              1¨¤????¨¢|¡¤??¨°o¨ª¨¢¡Â¨¢?/¡À??¡§
  vx = 2*(q1q3 - q0q2);												//???a???Dxyz¦Ì?¡À¨ª¨º?
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //?¨°¨¢?¨ªa?y?¨²?¨¤??¦Ì?¦Ì?2?¡¤??¨ª¨º??¨®2?
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //???¨®2???DD?y¡¤?
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//???¨®2?PIo¨®213£¤¦Ì?¨ª¨®?Y¨°?¡ê??¡ä213£¤¨¢?¦Ì??¡¥¨°?
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//?a¨¤?¦Ì?gz¨®¨¦¨®¨²??¨®D1?2a????DD???y?¨¢2¨²¨¦¨²?¡¥¨°?¡ê?¡À¨ª??3?¨¤¡ä¦Ì??¨ª¨º??y¡¤?¡Á????¨°¡Á???

  // integrate quaternion rate and normalise						   //???a??¦Ì??¡é¡¤?¡¤?3¨¬
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  roll_radian = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1); // roll
  pitch_radian = asin(-2*q1*q3 + 2*q0*q2); // pitch
	
	/*          ????????????                       */    
	/*??  http://baike.baidu.com/view/1239157.htm?fr=aladdin */
	
	//Xr = X_HMC * COS(angle.pitch/AtR) + Y_HMC * SIN(-angle.pitch/AtR) * SIN(-angle.roll/AtR) - Z_HMC * COS(angle.roll/AtR) * SIN(-angle.pitch/AtR);
	//Yr = Y_HMC * COS(angle.roll/AtR) + Z_HMC * SIN(-angle.roll/AtR);
	
	 // angle.yaw = atan2((double)Y_HMC,(double)X_HMC) * RtA; // yaw 
	  imu_data.yaw += gz *0.229183118f;//*0.00000425474;//0.004*0.0010636856
	 // angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* RtA;
	  
	  imu_data.rol =roll_radian * 57.30f;//y?
	  imu_data.pit=pitch_radian * 57.30f;//x?
	  
	
	//Send_data_float(angle.yaw,'Y');
	  //Send_data_float(sensor.gyro.radian.z,'z');
        //Send_data_float(sensor.gyro.radian.y ,'x');
	
	
	  // R_UART2_Send(ab,1);
	//  Send_data_float(ADNS_Data_x,'x');
	//Send_data_float(chao_meter,'r');
	//Send_data_int(fly_ud,'u');
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

