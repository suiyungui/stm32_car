#include "mcu_dmp.h"

/* ��̬�����㷨���� 
 * ���û�Ϸ�������ֹʱʹ��PI���������˶�ʱʹ��Madgwick�㷨
 * 
 * ��Ʈ������η�����
 * 1. ������ֹ��Ʈ
 *    - ���󣺿���ʱIMU��ֹ���ǶȻ���Ư��
 *    - ���������
 *      a) ����Ki��0.0001f-0.0003f
 *      b) �ʵ�����Kp��18.0f-20.0f
 *      c) ����STATIC_THRESHOLD��0.8f����ʹ�����׽��뾲ֹģʽ
 * 
 * 2. �˶���ֹ��Ʈ
 *    - �����˶�ֹͣ��Ƕȼ���Ư��
 *    - ���������
 *      a) �����������˲�������
 *         - ����q��0.001f��СԤ�����
 *         - ����r��0.2f���Ͳ������ζ�
 *      b) ����alpha��0.9f��ǿ�˲�
 *      c) ��СMAX_YAW_DELTA��6.0f���Ʊ仯��
 * 
 * 3. ����ת������Ʈ
 *    - ���󣺾����˶���ص���ֹ״̬��Ư��
 *    - ���������
 *      a) ����beta��0.08f��С��̬��Ӧ
 *      b) ����STATIC_THRESHOLD��0.8f
 *      c) ����MAX_YAW_RATE��60.0f���ƽ��ٶ�
 * 
 * 4. ��Ư����
 *    - ���󣺳�ʱ��ʹ�ú��𽥲���Ư��
 *    - ���������
 *      a) ��ȫ����Ki��0f������������
 *      b) ����r��0.25f���ͶԲ���������
 *      c) ����alpha��0.95f��ǿ��ʷ����Ȩ��
 * 
 * 5. �𶯻�����Ʈ
 *    - ���������𶯵Ļ����½Ƕ�Ư��
 *    - ���������
 *      a) ����STATIC_THRESHOLD��1.0f
 *      b) ����alpha��0.92f
 *      c) ����r��0.3f
 *      d) ��Сq��0.001f
 *
 * �������ȼ���
 * 1. �ȵ���STATIC_THRESHOLDȷ����ֹ���׼ȷ
 * 2. ����Kp/Ki�����ֹƯ��
 * 3. ����beta�����̬����
 * 4. ���΢���������͵�ͨ�˲�����
 * 
 * ���μ��ɣ�
 * 1. beta: Madgwick�㷨���棬Ӱ�춯̬��Ӧ�ٶ�
 *    - ����beta�����ٸ��죬�����ܸ�����
 *    - ��Сbeta����ƽ�ȣ�����̬��Ӧ����
 *    - ���鷶Χ��0.05f-0.15f
 *    - ��̬������������0.12f-0.15f
 *    - ��ȷ������������0.05f-0.08f
 * 
 * 2. Kp/Ki: PI��������������ҪӰ�쾲ֹ����
 *    - KpӰ������ٶȣ�KiӰ�쾲̬����
 *    - ����ʱ������Kp����СKi
 *    - Ư��ʱ������Kp���ʵ�����Ki
 *    - ���鷶Χ��Kp(10.0f-20.0f), Ki(0.0001f-0.001f)
 */
static float beta = 0.08f;                               /* Madgwick�㷨���� */
static float Kp = 15.00f;                               /* �������棨���ھ�ֹ״̬�� */
static float Ki = 0.0005f;                              /* �������棨���ھ�ֹ״̬�� */
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;  /* ��������ۼ� */


/* ��Ԫ������ʾ��ǰ��̬ */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* ��ת��������ŷ���Ǽ��� */
static float rMat[3][3];

/* ƫ���ǿ������˲���
 * ���μ��ɣ�
 * 1. q(��������)����ӳϵͳ��̬����
 *    - ����q����Ӧ���죬�������ȶ�
 *    - ��Сq�����ȶ�������Ӧ����
 *    - ����ת��������������0.005f
 *    - �ȶ���������С��0.001f
 * 
 * 2. r(��������)����ӳ�������Ŷ�
 *    - ����r����������Ԥ��ֵ����ƽ���������ͺ�
 *    - ��Сr��������������ֵ����Ӧ�쵫���ܶ���
 *    - �𶯻�����������0.2f-0.3f
 *    - �ȶ���������С��0.1f
 */
static KalmanFilter yaw_kf = {
    .q = 0.001f,    // ��������
    .r = 0.3f,     // ��������
    .x = 0.0f,      // ״̬����ֵ
    .p = 0.1f,      // �������Э����
    .k = 0.0f,      // ����������
    .init = 0       // ��ʼ����־
};

/* ״̬���� */
static float last_yaw_rate = 0.0f;
static float last_yaw = 0.0f;
static float filtered_yaw_rate = 0.0f;

/* �˲������Ʋ���
 * ���μ��ɣ�
 * 1. alpha����ͨ�˲�ϵ��
 *    - ����alpha����ƽ�������ͺ�����
 *    - ��Сalpha����Ӧ���죬�����ܸ�����
 *    - ���鷶Χ��0.7f-0.95f
 *    - ��Ƶ�𶯻�����������0.9f����
 *    - ������Ӧ��������С��0.7f-0.8f
 * 
 * 2. MAX_YAW_RATE��ƫ�����ٶ�����
 *    - ������������ת��
 *    - ��С�����õ��ȶ���
 *    - ���鷶Χ��50.0f-100.0f
 *    - ����ת��������������100.0f
 *    - ��ȷ���Ƴ�������С��50.0f
 * 
 * 3. MAX_YAW_DELTA��ƫ���Ǳ仯����
 *    - ������Ӧ����
 *    - ��С�����õĿ�������
 *    - ���鷶Χ��5.0f-15.0f
 *    - ��Χת����������12.0f-15.0f
 *    - ��ȷ���ƣ���С��5.0f-8.0f
 * 
 * 4. STATIC_THRESHOLD����ֹ�����ֵ
 *    - ���󣺸����׽��뾲ֹģʽ
 *    - ��С�������׽����˶�ģʽ
 *    - ���鷶Χ��0.3f-1.0f
 *    - �𶯻�����������0.8f-1.0f
 *    - �߾������󣺼�С��0.3f-0.5f
 */
static const float alpha = 0.85f;           // ��ͨ�˲�ϵ�� (0-1)
static const float MAX_YAW_RATE = 80.0f;    // ���ƫ�����ٶ� (��/��)
static const float MAX_YAW_DELTA = 8.0f;    // ���ƫ���Ǳ仯 (��)
static const float STATIC_THRESHOLD = 0.10f;  // ��ֹ�����ֵ

/* ����Ӧ�˲������ṹ�� */
typedef struct {
    float alpha_min;      // ��С�˲�ϵ��
    float alpha_max;      // ����˲�ϵ��
    float rate_threshold; // ���ٶ���ֵ
} AdaptiveFilter;

static AdaptiveFilter adaptive_filter = {
    .alpha_min = 0.75f,
    .alpha_max = 0.95f,
    .rate_threshold = 50.0f
};

/* ����Ӧ�˲�ϵ������ */
float calculate_adaptive_alpha(float yaw_rate) 
{
    float rate_abs = fabsf(yaw_rate);
    if(rate_abs < adaptive_filter.rate_threshold) 
		{
        return adaptive_filter.alpha_max - 
               (rate_abs / adaptive_filter.rate_threshold) * 
               (adaptive_filter.alpha_max - adaptive_filter.alpha_min);
    }
    return adaptive_filter.alpha_min;
}

/* ���ٿ�ƽ���󵹺��� */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* �������˲������� */
float kalman_filter(KalmanFilter* kf, float measurement, float gyro_rate, float dt)
{
    // ��ʼ��
    if(!kf->init) {
        kf->x = measurement;
        kf->init = 1;
        return kf->x;
    }
    
    // Ԥ�ⲽ��
    kf->x = kf->x + gyro_rate * dt;
    kf->p = kf->p + kf->q;
    
    // ���²���
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;
    
    return kf->x;
}

/* ��ʼ��IMU */
void imu_init(void)
{
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    
    // ��ʼ��ƫ���ǿ������˲���
    yaw_kf.x = 0.0f;
    yaw_kf.p = 0.1f;
    yaw_kf.init = 0;
}

/* ������ת���� */
static void compute_rotation_matrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

/* �����ں� - �����˲� */
void imu_update(Axis3f acc, Axis3f gyro, float dt)
{
    float normalise;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, q0q0, q1q1, q2q2, q3q3;
    float gyro_magnitude;

    /* ���ٶȵ�λת��Ϊ���� */
    gyro.x = gyro.x * DEG2RAD;
    gyro.y = gyro.y * DEG2RAD;
    gyro.z = gyro.z * DEG2RAD;

    /* ������ٶȴ�С���ھ�ֹ��� */
    gyro_magnitude = sqrtf(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);

    /* �����ٶȼ������Ƿ���Ч */
    if ((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
    {
        /* ��λ�����ٶȼ����� */
        normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
        acc.x *= normalise;
        acc.y *= normalise;
        acc.z *= normalise;

        /* �ж��Ƿ��ھ�ֹ״̬ */
        if (gyro_magnitude < STATIC_THRESHOLD) {
            /* ��ֹ״̬ʹ��PI������ */
            float ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
            float ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
            float ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);

            /* ��������ۻ� */
            exInt += Ki * ex * dt;
            eyInt += Ki * ey * dt;
            ezInt += Ki * ez * dt;

            /* Ӧ��PI���� */
            gyro.x += Kp * ex + exInt;
            gyro.y += Kp * ey + eyInt;
            gyro.z += Kp * ez + ezInt;
        } else {
            /* �˶�״̬ʹ��Madgwick�㷨 */
            /* ������Ԫ���仯�� */
            qDot1 = 0.5f * (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z);
            qDot2 = 0.5f * (q0 * gyro.x + q2 * gyro.z - q3 * gyro.y);
            qDot3 = 0.5f * (q0 * gyro.y - q1 * gyro.z + q3 * gyro.x);
            qDot4 = 0.5f * (q0 * gyro.z + q1 * gyro.y - q2 * gyro.x);

            /* �����ݶ� */
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            s0 = _2q0 * q2q2 + _2q2 * acc.x + _2q0 * q1q1 - _2q1 * acc.y;
            s1 = _2q1 * q3q3 - _2q3 * acc.x + 4.0f * q0q0 * q1 - _2q0 * acc.y - _2q1 + _2q1 * q1q1 + _2q1 * q2q2 + _2q1 * acc.z;
            s2 = 4.0f * q0q0 * q2 + _2q0 * acc.x + _2q2 * q3q3 - _2q3 * acc.y - _2q2 + _2q2 * q1q1 + _2q2 * q2q2 + _2q2 * acc.z;
            s3 = 4.0f * q1q1 * q3 - _2q1 * acc.x + 4.0f * q2q2 * q3 - _2q2 * acc.y;

            /* ��һ���ݶ� */
            normalise = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= normalise;
            s1 *= normalise;
            s2 *= normalise;
            s3 *= normalise;

            /* Ӧ���ݶ��½� */
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;

            /* ���ֵõ���Ԫ�� */
            q0 += qDot1 * dt;
            q1 += qDot2 * dt;
            q2 += qDot3 * dt;
            q3 += qDot4 * dt;
        }

        /* ��Ԫ����һ�� */
        normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= normalise;
        q1 *= normalise;
        q2 *= normalise;
        q3 *= normalise;

        /* ������ת���� */
        compute_rotation_matrix();
    }
}

typedef struct {
    float x[2];    // ״̬���� [�Ƕ�, ���ٶ�]
    float p[2][2]; // Э�������
    float q[2];    // ��������
    float r;       // ��������
    int init;
} ExtendedKalmanFilter;

static ExtendedKalmanFilter yaw_ekf = {
    .x = {0.0f, 0.0f},
    .p = {{0.1f, 0.0f}, {0.0f, 0.1f}},
    .q = {0.001f, 0.002f},
    .r = 0.3f,
    .init = 0
};


/* ��չ�������˲����� */
void ekf_update(ExtendedKalmanFilter* ekf, float measurement, float dt) 
{
    if(!ekf->init) 
		{
        ekf->x[0] = measurement;
        ekf->init = 1;
        return;
    }
    
    // Ԥ�ⲽ��
    float F[2][2] = {{1.0f, dt}, {0.0f, 1.0f}};
    ekf->x[0] += ekf->x[1] * dt;
    
    // ����Э����
    float temp_p[2][2];
    temp_p[0][0] = ekf->p[0][0] + dt * ekf->p[1][0] + dt * ekf->p[0][1] + dt * dt * ekf->p[1][1] + ekf->q[0];
    temp_p[0][1] = ekf->p[0][1] + dt * ekf->p[1][1];
    temp_p[1][0] = ekf->p[1][0] + dt * ekf->p[1][1];
    temp_p[1][1] = ekf->p[1][1] + ekf->q[1];
    
    // ���㿨��������
    float k[2];
    k[0] = temp_p[0][0] / (temp_p[0][0] + ekf->r);
    k[1] = temp_p[1][0] / (temp_p[0][0] + ekf->r);
    
    // ����״̬
    float innovation = measurement - ekf->x[0];
    ekf->x[0] += k[0] * innovation;
    ekf->x[1] += k[1] * innovation;
    
    // ����Э����
    ekf->p[0][0] = (1 - k[0]) * temp_p[0][0];
    ekf->p[0][1] = (1 - k[0]) * temp_p[0][1];
    ekf->p[1][0] = temp_p[1][0] - k[1] * temp_p[0][0];
    ekf->p[1][1] = temp_p[1][1] - k[1] * temp_p[0][1];
}

/* �쳣ֵ��� */
int is_yaw_outlier(float yaw, float last_yaw, float yaw_rate, float dt) 
{
    float predicted_yaw = last_yaw + yaw_rate * dt;
    float delta = fabsf(yaw - predicted_yaw);
    return delta > MAX_YAW_DELTA * 1.5f;
}


/* ��ȡŷ���� */
EulerAngles imu_get_euler_angles(Axis3f gyro)
{
    EulerAngles angles;
    float yaw_rate;
    static float last_raw_yaw = 0.0f;
    static float last_filtered_yaw = 0.0f;
    static float filtered_yaw_rate = 0.0f;
    
    // ����ת��������ȡŷ����
    angles.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
    angles.pitch = asinf(-rMat[2][0]) * RAD2DEG;
    
    // ����ԭʼƫ����
    float raw_yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
    
    // ����ƫ�����ٶȣ�ֱ��ʹ��������z�����ݣ�������̬������
    float cos_pitch = cosf(angles.pitch * DEG2RAD);
    float sin_pitch = sinf(angles.pitch * DEG2RAD);
    float cos_roll = cosf(angles.roll * DEG2RAD);
    float sin_roll = sinf(angles.roll * DEG2RAD);
    
    // �������ƫ�����ٶ�
    yaw_rate = (gyro.z * cos_roll * cos_pitch + 
                gyro.y * sin_roll * cos_pitch - 
                gyro.x * sin_pitch) * RAD2DEG;
    
    // ����ƫ�����ٶ�
    if(yaw_rate > MAX_YAW_RATE) yaw_rate = MAX_YAW_RATE;
    if(yaw_rate < -MAX_YAW_RATE) yaw_rate = -MAX_YAW_RATE;
    
    // ��ͨ�˲�����ƫ�����ٶ�
    float adaptive_alpha = calculate_adaptive_alpha(yaw_rate);
    filtered_yaw_rate = adaptive_alpha * filtered_yaw_rate + 
                    (1.0f - adaptive_alpha) * yaw_rate;
    
    // ʹ����չ�������˲�
    ekf_update(&yaw_ekf, raw_yaw, 0.005f);
    float kf_yaw = yaw_ekf.x[0];
    
    // ����Ӧ���׵�ͨ�˲�
    float beta1 = adaptive_alpha;
    float beta2 = adaptive_alpha;
    float temp_yaw = beta1 * last_yaw + (1.0f - beta1) * kf_yaw;
    angles.yaw = beta2 * last_filtered_yaw + (1.0f - beta2) * temp_yaw;
    
    // ������ʷֵ
    last_raw_yaw = raw_yaw;
    last_yaw = temp_yaw;
    last_filtered_yaw = angles.yaw;
    last_yaw_rate = yaw_rate;
    
    return angles;
} 



