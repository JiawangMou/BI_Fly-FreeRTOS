#include "system.h"
#include "vl53lxx_i2c_hw.h"
#include "ina226.h"
#include "vl53l1_api.h"
#include "i2cdev.h"
#include "math.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/*
    不确定具体电阻大射1�7
*/
INA226 INA226_data;
UOC uoc;
IOC ioc;
static float SOC = 1;
static float U1 = 0, U2 = 0;
static float dt = 0.1; //间隔时间，后面需要调敄1�7
static float Q_est = 0.133;
//以分段列表的形式写入拟合结果，从0.1-0.99分成90殄1�7
float USeg[90] = {3.6985, 3.7018, 3.7051, 3.7085, 3.7119, 3.7154, 3.7189, 3.7224, 3.726, 3.7296, 3.7333, 3.7369, 3.7407, 3.7444, 3.7483, 3.7521, 3.756, 3.7599, 3.7639, 3.7679, 3.7719, 3.776, 3.7802, 3.7843, 3.7886, 3.7928, 3.7972, 3.8015, 3.8059, 3.8104, 3.8149, 3.8195, 3.8241, 3.8288, 3.8335, 3.8383, 3.8432, 3.8481, 3.8531, 3.8581, 3.8633, 3.8685, 3.8737, 3.8791, 3.8845, 3.89, 3.8956, 3.9013, 3.9071, 3.9129, 3.9189, 3.925, 3.9311, 3.9374, 3.9438, 3.9502, 3.9568, 3.9636, 3.9704, 3.9774, 3.9844, 3.9917, 3.999, 4.0065, 4.0142, 4.022, 4.0299, 4.038, 4.0463, 4.0547, 4.0633, 4.072, 4.081, 4.0901, 4.0994, 4.1089, 4.1186, 4.1285, 4.1387, 4.149, 4.1595, 4.1703, 4.1813, 4.1925, 4.204, 4.2157, 4.2276, 4.2399, 4.2523, 4.2651};
float SSeg[90] = {0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.4, 0.41, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.5, 0.51, 0.52, 0.53, 0.54, 0.55, 0.56, 0.57, 0.58, 0.59, 0.6, 0.61, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68, 0.69, 0.7, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79, 0.8, 0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87, 0.88, 0.89, 0.9, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99};
float R1Seg[90] = {0.021872, 0.040934, 0.023881, 0.01873, 0.03573, 0.029658, 0.014023, 0.025463, 0.038367, 0.033305, 0.044102, 0.066855, 0.054383, 0.030409, 0.038651, 0.057402, 0.062629, 0.054306, 0.043278, 0.043563, 0.047991, 0.046697, 0.042293, 0.042653, 0.057191, 0.067552, 0.05093, 0.037813, 0.050298, 0.061433, 0.063097, 0.069303, 0.071277, 0.05354, 0.034786, 0.031188, 0.024099, 0.013444, 0.027053, 0.045981, 0.036916, 0.032859, 0.054527, 0.053126, 0.023887, 0.018919, 0.04004, 0.058068, 0.056641, 0.035866, 0.022973, 0.026684, 0.031853, 0.033076, 0.032125, 0.033964, 0.041274, 0.050109, 0.05835, 0.065966, 0.06879, 0.06255, 0.050107, 0.039838, 0.045664, 0.058309, 0.051698, 0.036696, 0.033592, 0.034976, 0.034304, 0.037024, 0.043754, 0.049911, 0.052896, 0.053206, 0.058161, 0.063077, 0.04452, 0.018906, 0.025197, 0.045131, 0.050345, 0.049083, 0.0473, 0.03845, 0.030258, 0.036912, 0.042499, 0.03574};
float R2Seg[90] = {0.084712, 0.063877, 0.081241, 0.091496, 0.082209, 0.09191, 0.10764, 0.096767, 0.08457, 0.089536, 0.078482, 0.056847, 0.073454, 0.10057, 0.09079, 0.074386, 0.078219, 0.09078, 0.097467, 0.08903, 0.0843, 0.097053, 0.10393, 0.088667, 0.053732, 0.029644, 0.046879, 0.065377, 0.053384, 0.038312, 0.02976, 0.015268, 0.0087146, 0.030515, 0.055827, 0.060851, 0.062299, 0.070319, 0.077715, 0.080142, 0.075166, 0.058153, 0.035303, 0.037509, 0.063683, 0.074854, 0.061729, 0.038285, 0.032173, 0.053963, 0.069953, 0.07008, 0.074936, 0.079898, 0.070632, 0.061279, 0.062404, 0.059698, 0.044355, 0.018168, -0.00015053, 0.0089266, 0.038651, 0.067329, 0.060344, 0.032472, 0.035896, 0.05565, 0.057425, 0.049854, 0.044765, 0.042593, 0.038072, 0.024291, 0.011406, 0.0090981, 0.0027254, -0.0049554, 0.020707, 0.057507, 0.050536, 0.025423, 0.02102, 0.021755, 0.019675, 0.031104, 0.043142, 0.031434, 0.042733, 0.084175};
float C1Seg[90] = {317.86, 353.28, 201.03, 103.25, 218.86, 527.11, 736.74, 542.28, 286.29, 285.66, 378.75, 436.32, 445.46, 431.04, 422.81, 408.62, 388.01, 394.59, 409.91, 387.98, 379.9, 424.97, 447.88, 436.15, 465.52, 480.3, 398.14, 377.53, 513.95, 625.62, 578.75, 346.3, 147.25, 238.54, 374.03, 333.49, 380.56, 556.35, 536.45, 385.15, 349.43, 392.2, 428.57, 453.84, 476.76, 503.56, 479.93, 359.16, 296.73, 385.63, 465.37, 477.35, 504.6, 524.45, 474.78, 421.06, 444.47, 580.44, 863.85, 1287.2, 1466.4, 1076.5, 673.43, 617.47, 583.49, 486.11, 489.7, 514.23, 435.91, 387.51, 438.68, 455.54, 417.42, 404.8, 357.79, 261.71, 435.94, 854.78, 877.59, 545.09, 373.36, 336.79, 270.25, 232.23, 251.9, 261.59, 314.06, 494.96, 518.26, 415.18};
float C2Seg[90] = {48.764, 60.813, 83.629, 81.174, 51.33, 26.088, 19.607, 29.997, 34.974, 27.973, 48.813, 84.289, 65.586, 30.065, 42.612, 61.9, 53.323, 37.662, 30.435, 36.002, 43.444, 40.722, 27.14, 29.712, 96.34, 152.82, 98.794, 41.455, 72.693, 128.82, 179.41, 238.96, 253.36, 160.08, 56.562, 34.031, 34.079, 23.896, 33.253, 47.605, 40.236, 60.791, 124.26, 121.29, 44.869, 16.025, 58.682, 124.52, 138.31, 75.219, 32.882, 42.392, 46.882, 40.476, 43.792, 48.03, 50.118, 75.558, 110.06, 108.21, 114.25, 172.65, 169.16, 74.324, 62.091, 129.77, 119.7, 63.493, 51.904, 67.934, 80.317, 76.826, 85.285, 160.5, 304.49, 486.86, 644.64, 667.78, 405.08, 56.725, 9.174, 119.25, 140.56, 136, 147.73, 109.27, 61.451, 97.998, 105.38, 12.235};
float R0Seg[90] = {0.18341, 0.18296, 0.18329, 0.18482, 0.18616, 0.18433, 0.18202, 0.18371, 0.18637, 0.18695, 0.18721, 0.18789, 0.18805, 0.188, 0.1885, 0.18918, 0.18931, 0.18815, 0.187, 0.18817, 0.1903, 0.19127, 0.19041, 0.18847, 0.18747, 0.18796, 0.18933, 0.18999, 0.18929, 0.18904, 0.18976, 0.1905, 0.19097, 0.19123, 0.1909, 0.19005, 0.19035, 0.19176, 0.19187, 0.19103, 0.191, 0.1924, 0.19421, 0.19274, 0.1893, 0.19065, 0.1949, 0.19616, 0.19604, 0.19669, 0.19705, 0.19693, 0.19749, 0.19801, 0.1973, 0.19714, 0.19851, 0.19901, 0.19882, 0.20052, 0.202, 0.20042, 0.19875, 0.19931, 0.20088, 0.20263, 0.20433, 0.20499, 0.20392, 0.20279, 0.20279, 0.20327, 0.20388, 0.20472, 0.20613, 0.20785, 0.20753, 0.20534, 0.20577, 0.20862, 0.21025, 0.20989, 0.20841, 0.20824, 0.21057, 0.21428, 0.21612, 0.21329, 0.21373, 0.21378};
float *Coe, Q;
arm_matrix_instance_f32 Matrix_Q;

float theta[6] = {0.5, 0.3, 0.2, 0.1, 0.1, 0.1};
arm_matrix_instance_f32 Matrixtheta;
float P[36] = {100.f, 0.f, 0.f, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100};
float P3[36] = {100.f, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 100};
arm_matrix_instance_f32 MatrixP, Matrix_P3;
float phi[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
arm_matrix_instance_f32 Matrixphi, MatrixphiT;
float P1[6] = {0, 0, 0, 0, 0, 0};
float P2[6] = {0, 0, 0, 0, 0, 0};
arm_matrix_instance_f32 Matrix_P1, Matrix_P2;
float K[6] = {0, 0, 0, 0, 0, 0};
arm_matrix_instance_f32 Matrix_K, Matrix_KT;
arm_matrix_instance_f32 Matrix_Uest, Matrix_Uerror;
u8 key, len;
u16 iSOCtest = 0, ID, Itemp, calibrated, configuration;
bool right = true;
float U, s_vol, powerval, U_error;
float SOC_temp;
float R1SOC, R2SOC, C1SOC, C2SOC, R0SOC, USOC;
float Ureal;
extern uint8_t ina226config[4] = {0xDF, 0x44, 0x0A, 0x00};
void INA226_Init(void)
{
    // vl53_HWIIC_Init();
    // delay_us(10);
    i2cdevWrite(I2C1_DEV, INA226_ADDR1, CFG_REG, 2, &ina226config[0]);
    i2cdevWrite(I2C1_DEV, INA226_ADDR1, CAL_REG, 2, &ina226config[2]);
}

void INA226_GET_ALL(u8 addr)
{
    //
}
void PreSet(u32 *time)
{
    uoc.actualVal = INA226_GetVoltage(INA226_ADDR1)/1000.f;
    ioc.cur = INA226_GetShunt_Current(INA226_ADDR1)/1000.f;
    vTaskDelayUntil(time, 10);
    uoc.lastVal = uoc.actualVal;
    ioc.last_cur = ioc.cur;
    uoc.actualVal = INA226_GetVoltage(INA226_ADDR1);
    ioc.cur = INA226_GetShunt_Current(INA226_ADDR1);
    vTaskDelayUntil(time, 10);
    uoc.llastVal = uoc.lastVal;
    ioc.llast_cur = ioc.llast_cur;
    uoc.lastVal = uoc.actualVal;
    ioc.last_cur = ioc.cur;
    uoc.actualVal = INA226_GetVoltage(INA226_ADDR1)/1000.f;
    ioc.cur = INA226_GetShunt_Current(INA226_ADDR1)/1000.f;
}
void LRS_RC2(float S)
{
    int RC_i = 0;
    phi[0] = findCoe(S)[0];
    phi[1] = uoc.lastVal;
    phi[2] = uoc.llastVal;
    phi[3] = ioc.cur;
    phi[4] = ioc.last_cur;
    phi[5] = ioc.llast_cur;

    arm_mat_mult_f32(&MatrixP, &Matrixphi, &Matrix_P1);
    arm_mat_mult_f32(&MatrixphiT, &Matrix_P1, &Matrix_Q);
    arm_mat_mult_f32(&MatrixP, &Matrixphi, &Matrix_K);

    for (RC_i = 0; RC_i < 6; RC_i++)
    {
        K[RC_i] = K[RC_i] / (Q+1);
    }

    arm_mat_mult_f32(&Matrixtheta, &Matrixphi, &Matrix_Uest);
    U_error = uoc.actualVal - uoc.estiVal;
    arm_mat_mult_f32(&MatrixphiT, &MatrixP, &Matrix_P2);
    arm_mat_mult_f32(&Matrix_K, &Matrix_P2, &Matrix_P3);
    for (RC_i = 0; RC_i < 36; RC_i++)
    {
        P[RC_i] -= P3[RC_i];
    }
    for (RC_i = 0; RC_i < 6; RC_i++)
    {
        theta[RC_i] += K[RC_i] * U_error;
    }
}
void Matrix_Init()
{
    arm_mat_init_f32(&Matrixtheta, 1, 6, theta);
    arm_mat_init_f32(&MatrixP, 6, 6, P);

    arm_mat_init_f32(&Matrixphi, 6, 1, phi);
    arm_mat_init_f32(&MatrixphiT, 1, 6, phi);
    arm_mat_init_f32(&Matrix_P1, 6, 1, P1);
    arm_mat_init_f32(&Matrix_P2, 1, 6, P2);
    arm_mat_init_f32(&Matrix_Q, 1, 1, &Q);
    arm_mat_init_f32(&Matrix_K, 6, 1, K);
    arm_mat_init_f32(&Matrix_Uest, 1, 1, &(uoc.estiVal));
    arm_mat_init_f32(&Matrix_Uerror, 1, 1, &U_error);
    arm_mat_init_f32(&Matrix_P3, 6, 6, P3);
}
void ina226loop()
{
    i2cdevWrite(I2C1_DEV, INA226_ADDR1, CAL_REG, 2, &ina226config[2]);
	GetVoltage(&INA226_data.voltageVal); 
    GetPower();
    ioc.cur = INA226_data.Shunt_Current;
    uoc.actualVal = INA226_data.voltageVal;
    SOC -= 0.001f * ioc.cur * dt / (3600 * Q_est);
    LRS_RC2(SOC);
    ioc.llast_cur = ioc.last_cur;
    ioc.last_cur = ioc.cur;
    uoc.llastVal = uoc.lastVal;
    uoc.lastVal = uoc.actualVal;
}
void ina226Task(void *arg)
{
    u32 lastWakeTime = getSysTickCnt();
    Matrix_Init();
    PreSet(&lastWakeTime);
    INA226_Init();
    ID = INA226_Get_ID(INA226_ADDR1);
    calibrated = INA226_GET_CAL_REG(INA226_ADDR1);
    configuration = INA226_Get_CFG_REG(INA226_ADDR1);
    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, 10); // 10ms周期延时
        ina226loop();
        // phi[1] =
        //  s_cur = INA226_data.Shunt_Current;
        //  Coe = findCoe(SOC);
        //  USOC = Coe[0];
        //  R0SOC = Coe[1];
        //  R1SOC = Coe[2];
        //  C1SOC = Coe[3];
        //  R2SOC = Coe[4];
        //  C2SOC = Coe[5];
        //  if (s_cur == 0)
        //  {
        //      SOC = findSOC(voltage);
        //  }
        //  SOC = SOC - 0.01f * s_cur * dt / (3600 * Q_est);

        // U1 = U1 * (float)(exp((double)(-dt / (R1SOC * C1SOC)))) + s_cur * R1SOC * (1 - (float)(exp((double)(-dt / (R1SOC * C1SOC)))));
        // U2 = U2 * (float)(exp((double)(-dt / (R2SOC * C2SOC)))) + s_cur * R2SOC * (1 - (float)(exp((double)(-dt / (R2SOC * C2SOC)))));
        // Ureal = voltage + bus_cur * R0SOC + U1 + U2;
        // if (findSOC(Ureal / 1000) < 0.9f)
        // {
        //     right = false;
        // }
        // tick++;
        // vTaskDelayUntil(&xLastWakeTime, 50);
    }
}
void getPower(float *p)
{
    *p = powerval;
}
void getSOC(float *soc)
{
    *soc = SOC;
}
void getVoltage(UOC* u)
{
    u->actualVal = uoc.actualVal;
    u->estiVal = uoc.estiVal;
    u->lastVal = uoc.lastVal;
    u->llastVal = uoc.llastVal;
}
// void getvoltage(float *v)
// {
//     *v = s_vol;
// }
// void getcurrent(float *c)
// {
//     *c = s_cur;
// }
float findSOC(float U)
{
    int i = 0;
    float s = 0.1;
    for (i = 0; i < 88; i++)
    {
        if (U > USeg[i] && U <= USeg[i + 1])
            break;
    }
    s = SSeg[i];
    return s;
}
float *findCoe(float S)
{
    float a[6] = {USeg[0], R0Seg[0], R1Seg[0], C1Seg[0], R2Seg[0], C2Seg[0]};
    int i = 0;
    for (i = 0; i < 88; i++)
    {
        if (S > SSeg[i] && S <= SSeg[i + 1])
            break;
    }
    a[0] = USeg[i];
    a[1] = R0Seg[i];
    a[2] = R1Seg[i];
    a[3] = C1Seg[i];
    a[4] = R2Seg[i];
    a[5] = C2Seg[i];
    return a;
}

// 1mA/bit
u16 INA226_Get_CFG_REG(u8 addr)
{
    u8 temp[2];
    u16 ans = 0;
    i2cdevRead(I2C1_DEV, addr, CFG_REG, 2, temp);
    ans = (u16)(temp[0]);
    ans = (ans << 8) | temp[1];
    return ans;
}
u16 INA226_GetShunt_Current(u8 addr)
{
    u8 temp[2];
    u16 ans = 0;
    i2cdevRead(I2C1_DEV, addr, CUR_REG, 2, temp);
    ans = (u16)(temp[0]);
    ans = (ans << 8) | temp[1];
    return ans;
}

//获取 id
u16 INA226_Get_ID(u8 addr)
{
    u8 temp[2];
    u16 ans = 0;
    i2cdevRead(I2C1_DEV, addr, INA226_GET_ADDR, 2, temp);
    //    ans = ((*temp)<<8)+*(temp+1);
    ans = (u16)(*temp);
    ans = (ans << 8) | (*(temp + 1));
    return ans;
}

//获取校准倄1�71ￄ1�77
u16 INA226_GET_CAL_REG(u8 addr)
{

    u8 temp[2];
    u16 ans = 0;
    i2cdevRead(I2C1_DEV, addr, CAL_REG, 2, temp);
    ans = (temp[0] << 8) + *(temp + 1);
    return ans;
}

// 1.25mV/bit
u16 INA226_GetVoltage(u8 addr)
{
    u8 temp[2];
    u16 ans = 0;
    i2cdevRead(I2C1_DEV, addr, BV_REG, 2, &temp[0]);
    ans = (temp[0] << 8) + (temp[1]);
    return ans;
}

// 2.5uV/bit,感觉这个值是测不准的，所以下面改戄1�71ￄ1�772.2亄1�71ￄ1�77
u16 INA226_GetShuntVoltage(u8 addr)
{
    u8 temp[2];
    u16 ans = 0;
    i2cdevRead(I2C1_DEV, addr, SV_REG, 2, &temp[0]);
    ans = (temp[0] << 8) | (temp[1]);
    return ans;
}

u16 INA226_Get_Power(u8 addr)
{
    u8 temp[2];
    u16 ans = 0;
    i2cdevRead(I2C1_DEV, addr, PWR_REG, 2, temp);
    ans = (temp[0] << 8) + (temp[1]);
    return ans;
}

void GetVoltage(float *Voltage) // mV
{
    Voltage[0] = INA226_GetVoltage(INA226_ADDR1) * 1.25f;
}

void Get_Shunt_voltage(float *Voltage) // uV
{
    Voltage[0] = (INA226_GetShuntVoltage(INA226_ADDR1) * 2.2); //这里原来乘的系数昄1�71ￄ1�772.5
}

void Get_Shunt_Current(float *Current) // mA
{
    Current[0] = (INA226_GetShunt_Current(INA226_ADDR1) * 2.5f);
}
void Get_Power(float *Current) // W
{
    Current[0] = (INA226_Get_Power(INA226_ADDR1) * 50);
}

void GetPower() // W
{
    GetVoltage(&INA226_data.voltageVal);                                                         // mV
    Get_Shunt_voltage(&INA226_data.Shunt_voltage);                                               // uV
    Get_Shunt_Current(&INA226_data.Shunt_Current);                                               // mA
    INA226_data.powerVal = INA226_data.voltageVal * 0.001f * INA226_data.Shunt_Current * 0.001f; // mW
}

void getcurrent(float *c)
{
    *c = ioc.cur;
}
