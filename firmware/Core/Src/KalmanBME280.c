

#include "KalmanBME280.h"
#include "math.h"


//#if USE_MS5611
//#define KF_Z_MEAS_VARIANCE            200
#define KF_Z_MEAS_VARIANCE            150


#define KF_A_MEAS_VARIANCE   		50.0f
#define KF_ACCELBIAS_VARIANCE   0.005f

//#define KF_A_MEAS_VARIANCE   		50.0f
//#define KF_ACCELBIAS_VARIANCE   0.005f


float Pzz,Pzv,Pza,Pzb;
float Pvz,Pvv,Pva,Pvb;
float Paz,Pav,Paa,Pab;
float Pbz,Pbv,Pba,Pbb;

float AccelVariance; 
float ABiasVariance; 
float ZMeasVariance; 
float AMeasVariance; 
float KAdapt;



float z; // altitude
float v; // climb/sink rate
float a; // gravity-compensated net earth-z axis acceleration
float b; // acceleration residual bias (post-calibration)


void kalmanFilter4d_configure(float aVariance,  float kAdapt, float zInitial, float vInitial, float aInitial){
	ZMeasVariance = KF_Z_MEAS_VARIANCE;
	AMeasVariance = KF_A_MEAS_VARIANCE;
  ABiasVariance = KF_ACCELBIAS_VARIANCE;
	//AccelVariance = aVariance;
	AccelVariance = 120;
	
	KAdapt = kAdapt;

	z = zInitial;
	v = vInitial;
  a = aInitial;
	b = 0.0f; // assume residual acceleration bias = 0 initially

	Pzz = 1500.0f;  //Pzz = 1500.0f;
  Pzv = 0.0f;     //Pzv = 0.0f;   
	Pza = 0.0f;     //Pza = 0.0f;   
	Pzb = 0.0f;     //Pzb = 0.0f;   
	
	Pvz = Pzv; 
	Pvv = 1500.0f;
	Pva = 0.0f;
	Pvb = 0.0f;
	
	Paz = Pza;
	Pav = Pva;
	Paa = 100000.0f;//Paa = 100000.0f;
	Pab = 0.0f;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = 1500.0f;
	}

	void kalmanFilter4d_predict(float dt) {
	// Predicted (a priori) state vector estimate x_k- = F * x_k-1+
	float accel_true = a - b; // true acceleration = acceleration minus acceleration sensor bias
	z = z + (v * dt) + (accel_true * dt * dt* 0.5f);
	v = v + (accel_true * dt);

	// Predicted (a priori) state covariance estimate P_k- = (F * P_k-1+ * F_t) + Qk
	float dt2 = dt*dt;  // dt^2
	float dt3 = dt2*dt; // dt^3 
	float dt4 = dt2*dt2; // dt^4;
	float dt2div2 = dt2*0.5f; // dt^2/2
	float dt3div2 = dt3*0.5f; // dt^3/2
	float dt4div2 = dt4*0.5f; // dt^4/2
	float dt4div4 = dt4*0.25f; // dt^4/4
	
	float p00 = Pzz + 2.0f*Pzv*dt + (Pza - Pzb)*dt2  + Pvv*dt2div2 + (Pva - Pvb)*dt3 + (Paa+Pbb)*dt4div4 - Pab*dt4div2;
	float p01 = Pzv + dt*(Pza - Pzb + Pvv) + 3.0f*dt2div2*(Pva - Pvb) - Pab*dt3 + (Paa + Pbb)*dt3div2;
	float p02 = Pza + Pva*dt + (Paa - Pba)*dt2div2;
	float p03 = Pzb + Pvb*dt + (Pab - Pbb)*dt2div2;

	float p11 = Pvv + 2.0f*dt*(Pva - Pvb) + dt2*(Paa - 2.0f*Pab + Pbb);
	float p12 = Pva + dt*(Paa - Pba);
	float p13 = Pvb + dt*(Pab - Pbb);

	float p22 = Paa;
	float p23 = Pab;
	float p33 = Pbb;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 

	// Add Q_k
	Paa = Paa + AccelVariance;
	Pbb = Pbb + ABiasVariance;
	}
	
	float z_err;
	float a_err;
	
	
	
float kalmanFilter4d_update(float zm, float am, float* pz, float* pv) {
	// Innovation Error y_k = measurement minus apriori estimate
	z_err = zm - z;
	a_err = am - a;

	// Innovation covariance S_k
	// S_k = (H * P_k- * H_t) + R_k
	float s00 = Pzz;
	float s01 = Pza;
	float s10 = s01;
	float s11 = Paa;

	// add R_k
	s00 += ZMeasVariance;
	s11 += AMeasVariance;
	// inject additional uncertainty for high acceleration situations
	float accel_ext = fabs(am-b);//   abs() --> fabs()
	s11 += (KAdapt*accel_ext*accel_ext);

	// allow system to update acceleration sensor bias estimate only when there is low acceleration
	ABiasVariance = KF_ACCELBIAS_VARIANCE/(1.0f + accel_ext);	

	// Compute S_k_inv
	float sdetinv = 1.0f/(s00*s11 - s10*s01);
	float sinv00 = sdetinv * s11;
	float sinv01 = -sdetinv * s10;
	float sinv10 = sinv01;
	float sinv11 = sdetinv * s00;

	// Kalman gain K_k [4x2] matrix
	// K_k = P_k- * H_t * S_k_inv
	float k00 = Pzz*sinv00 + Pza*sinv10;  
	float k01 = Pzz*sinv01 + Pza*sinv11;
	float k10 = Pvz*sinv00 + Pva*sinv10;
	float k11 = Pvz*sinv01 + Pva*sinv11;
	float k20 = Paz*sinv00 + Paa*sinv10;
	float k21 = Paz*sinv01 + Paa*sinv11;
	float k30 = Pbz*sinv00 + Pba*sinv10;
	float k31 = Pbz*sinv01 + Pba*sinv11;

	// Updated (a posteriori) state estimate x_k+ 
	// x_k+ = x_k- + K_k * y_k
	z = z + (k00*z_err + k01*a_err);
	v = v + (k10*z_err + k11*a_err);
	a = a + (k20*z_err + k21*a_err);
	b = b + (k30*z_err + k31*a_err);

	// Updated (a posteriori) state covariance estimate P_k+
	// P_k+ = (I - K_k * H_k)*P_k-
	float tmp = 1.0f - k00;
	float p00 = tmp*Pzz - k01*Paz;
	float p01 = tmp*Pzv - k01*Pav;
	float p02 = tmp*Pza - k01*Paa;
	float p03 = tmp*Pzb - k01*Pab;

	float p11 = -k10*Pzv + Pvv - k11*Pav;
	float p12 = -k10*Pza + Pva - k11*Paa;
	float p13 = -k10*Pzb + Pvb - k11*Pab;

	float p22 = -k20*Pza + (1.0f-k21)*Paa;
	float p23 = -k20*Pzb + (1.0f-k21)*Pab;

	float p33 = -k30*Pzb -k31*Pab + Pbb;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 

	// return the state variables of interest (z and v)
	*pz = z;
	*pv = v;
	return z;
}
	
	