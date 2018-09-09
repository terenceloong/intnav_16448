#include <math.h>
#include <string.h>
#include "inertial.h"
#include "axes.h"
#include "matrix.h"

void acc2hor(float acc[3], double *theta, double *gamma) //rad
{
    double x, y, z, g;
	
	g = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]); //Avoid that asin can't be calculated.
    x = acc[0] / g;
    y = acc[1] / g;
    z = acc[2] / g;
	if(x > 1.0)
		x = 1.0;
	else if(x < -1.0)
		x= -1.0;	
    *theta = asin(x);
    *gamma = atan2(-y, -z);
}

void mag2psi(float mag[3], double theta, double gamma, double *psi) //rad
{
    double sin_gamma, cos_gamma;

    sin_gamma = sin(gamma);
    cos_gamma = cos(gamma);

    *psi = atan2(mag[2]*sin_gamma - mag[1]*cos_gamma,\
				 mag[0]*cos(theta) + (mag[2]*cos_gamma + mag[1]*sin_gamma)*sin(theta));
}

void accmag2att(float acc[3], float mag[3], double *psi, double *theta, double *gamma)
{
    acc2hor(acc, theta, gamma);
    mag2psi(mag, *theta, *gamma, psi);
}

void imu_compensate(IMU_DATA *imu, IMU_BIAS *bias)
{
    float magx, magy, magz;
    
    imu->gyro[0] -= bias->gyro[0];
    imu->gyro[1] -= bias->gyro[1];
    imu->gyro[2] -= bias->gyro[2];
    imu->acc[0] -= bias->acc[0];
    imu->acc[1] -= bias->acc[1];
    imu->acc[2] -= bias->acc[2];
    magx = imu->mag[0] - bias->mag_hard[0];
    magy = imu->mag[1] - bias->mag_hard[1];
    magz = imu->mag[2] - bias->mag_hard[2];
    imu->mag[0] = magx*bias->mag_soft[0][0] + magy*bias->mag_soft[0][1] + magz*bias->mag_soft[0][2];
    imu->mag[1] = magx*bias->mag_soft[1][0] + magy*bias->mag_soft[1][1] + magz*bias->mag_soft[1][2];
    imu->mag[2] = magx*bias->mag_soft[2][0] + magy*bias->mag_soft[2][1] + magz*bias->mag_soft[2][2];
}

#define R2D                 57.29577951308
#define D2R                 0.01745329251994
#define PI                  3.14159265359
#define PI_M2               6.28318530718
#define SOLVE_STEP          0.02
#define SOLVE_STEP_D2       0.01
#define SOLVE_STEP_SQ_D4    0.0001

void att_solve(IMU_DATA *imu, INER_NAV *iner, ATT_FILTER *filter, float *para)
{
    static double w0[3], w1[3], w2[3]; //rad
    double angle[3]; //rad
    double q1, q2, q3, q4, q, g;
    double K1[4], K2[4], K3[4], K4[4];
    double Phi[7][7], Gamma[7][6], Q[6], H[4][7], K[7][4];
    double P1[7][7], P2[7][7], P3[7][6];
    double PH1[7][4], PH2[4][4], PH3[4][4];
    double u, v, w;
    double dgx, dgy, dgz, dpsi;
    uint8_t update = 0;

    if(filter->flag == 2)
    {
        filter->flag = 1;
        filter->cnt++;
        /* Save omega */
        w2[0] = (double)imu->gyro[0] * D2R - filter->X[4];
        w2[1] = (double)imu->gyro[1] * D2R - filter->X[5];
        w2[2] = (double)imu->gyro[2] * D2R - filter->X[6];

        /* Use accelerometer and magnetometer output to calculate attitude */
        accmag2att(imu->acc, imu->mag, &angle[0], &angle[1], &angle[2]);
        iner->att_am[0] = (float)(angle[0] * R2D);
        iner->att_am[1] = (float)(angle[1] * R2D);
        iner->att_am[2] = (float)(angle[2] * R2D);

        /* Differential equations of quaternions */
        q1 = filter->X[0];
        q2 = filter->X[1];
        q3 = filter->X[2];
        q4 = filter->X[3];
        K1[0] = (-w0[0]*q2 - w0[1]*q3 - w0[2]*q4) * 0.5;
        K1[1] = ( w0[0]*q1 + w0[2]*q3 - w0[1]*q4) * 0.5;
        K1[2] = ( w0[1]*q1 - w0[2]*q2 + w0[0]*q4) * 0.5;
        K1[3] = ( w0[2]*q1 + w0[1]*q2 - w0[0]*q3) * 0.5;
        q1 = filter->X[0] + K1[0] * SOLVE_STEP_D2;
        q2 = filter->X[1] + K1[1] * SOLVE_STEP_D2;
        q3 = filter->X[2] + K1[2] * SOLVE_STEP_D2;
        q4 = filter->X[3] + K1[3] * SOLVE_STEP_D2;
        K2[0] = (-w1[0]*q2 - w1[1]*q3 - w1[2]*q4) * 0.5;
        K2[1] = ( w1[0]*q1 + w1[2]*q3 - w1[1]*q4) * 0.5;
        K2[2] = ( w1[1]*q1 - w1[2]*q2 + w1[0]*q4) * 0.5;
        K2[3] = ( w1[2]*q1 + w1[1]*q2 - w1[0]*q3) * 0.5;
        q1 = filter->X[0] + K2[0] * SOLVE_STEP_D2;
        q2 = filter->X[1] + K2[1] * SOLVE_STEP_D2;
        q3 = filter->X[2] + K2[2] * SOLVE_STEP_D2;
        q4 = filter->X[3] + K2[3] * SOLVE_STEP_D2;
        K3[0] = (-w1[0]*q2 - w1[1]*q3 - w1[2]*q4) * 0.5;
        K3[1] = ( w1[0]*q1 + w1[2]*q3 - w1[1]*q4) * 0.5;
        K3[2] = ( w1[1]*q1 - w1[2]*q2 + w1[0]*q4) * 0.5;
        K3[3] = ( w1[2]*q1 + w1[1]*q2 - w1[0]*q3) * 0.5;
        q1 = filter->X[0] + K2[0] * SOLVE_STEP;
        q2 = filter->X[1] + K2[1] * SOLVE_STEP;
        q3 = filter->X[2] + K2[2] * SOLVE_STEP;
        q4 = filter->X[3] + K2[3] * SOLVE_STEP;
        K4[0] = (-w2[0]*q2 - w2[1]*q3 - w2[2]*q4) * 0.5;
        K4[1] = ( w2[0]*q1 + w2[2]*q3 - w2[1]*q4) * 0.5;
        K4[2] = ( w2[1]*q1 - w2[2]*q2 + w2[0]*q4) * 0.5;
        K4[3] = ( w2[2]*q1 + w2[1]*q2 - w2[0]*q3) * 0.5;
        q1 = filter->X[0] + (K1[0] + 2.0*K2[0] + 2.0*K3[0] + K4[0]) * SOLVE_STEP/6.0;
        q2 = filter->X[1] + (K1[1] + 2.0*K2[1] + 2.0*K3[1] + K4[1]) * SOLVE_STEP/6.0;
        q3 = filter->X[2] + (K1[2] + 2.0*K2[2] + 2.0*K3[2] + K4[2]) * SOLVE_STEP/6.0;
        q4 = filter->X[3] + (K1[3] + 2.0*K2[3] + 2.0*K3[3] + K4[3]) * SOLVE_STEP/6.0;

        /*--------------------------------EKF--------------------------------*/
        g = sqrt(imu->acc[0]*imu->acc[0] + imu->acc[1]*imu->acc[1] + imu->acc[2]*imu->acc[2]);
        if((0.988 < g) && (g < 1.012))
            update = 1;

        /* Time update */
        Phi[0][0] = 1.0;
        Phi[0][1] = SOLVE_STEP_D2 * -w1[0];
        Phi[0][2] = SOLVE_STEP_D2 * -w1[1];
        Phi[0][3] = SOLVE_STEP_D2 * -w1[2];
        Phi[0][4] = SOLVE_STEP_D2 *  q2;
        Phi[0][5] = SOLVE_STEP_D2 *  q3;
        Phi[0][6] = SOLVE_STEP_D2 *  q4;

        Phi[1][0] = SOLVE_STEP_D2 *  w1[0];
        Phi[1][1] = 1.0;
        Phi[1][2] = SOLVE_STEP_D2 *  w1[2];
        Phi[1][3] = SOLVE_STEP_D2 * -w1[1];
        Phi[1][4] = SOLVE_STEP_D2 * -q1;
        Phi[1][5] = SOLVE_STEP_D2 *  q4;
        Phi[1][6] = SOLVE_STEP_D2 * -q3;

        Phi[2][0] = SOLVE_STEP_D2 *  w1[1];
        Phi[2][1] = SOLVE_STEP_D2 * -w1[2];
        Phi[2][2] = 1.0;
        Phi[2][3] = SOLVE_STEP_D2 *  w1[0];
        Phi[2][4] = SOLVE_STEP_D2 * -q4;
        Phi[2][5] = SOLVE_STEP_D2 * -q1;
        Phi[2][6] = SOLVE_STEP_D2 *  q2;

        Phi[3][0] = SOLVE_STEP_D2 *  w1[2];
        Phi[3][1] = SOLVE_STEP_D2 *  w1[1];
        Phi[3][2] = SOLVE_STEP_D2 * -w1[0];
        Phi[3][3] = 1.0;
        Phi[3][4] = SOLVE_STEP_D2 *  q3;
        Phi[3][5] = SOLVE_STEP_D2 * -q2;
        Phi[3][6] = SOLVE_STEP_D2 * -q1;

        Phi[4][0] = 0.0;
        Phi[4][1] = 0.0;
        Phi[4][2] = 0.0;
        Phi[4][3] = 0.0;
        Phi[4][4] = 1.0;
        Phi[4][5] = 0.0;
        Phi[4][6] = 0.0;

        Phi[5][0] = 0.0;
        Phi[5][1] = 0.0;
        Phi[5][2] = 0.0;
        Phi[5][3] = 0.0;
        Phi[5][4] = 0.0;
        Phi[5][5] = 1.0;
        Phi[5][6] = 0.0;

        Phi[6][0] = 0.0;
        Phi[6][1] = 0.0;
        Phi[6][2] = 0.0;
        Phi[6][3] = 0.0;
        Phi[6][4] = 0.0;
        Phi[6][5] = 0.0;
        Phi[6][6] = 1.0;

        Gamma[0][0] = SOLVE_STEP_D2 * -q2;
        Gamma[0][1] = SOLVE_STEP_D2 * -q3;
        Gamma[0][2] = SOLVE_STEP_D2 * -q4;
        Gamma[0][3] = SOLVE_STEP_SQ_D4 * -q2;
        Gamma[0][4] = SOLVE_STEP_SQ_D4 * -q3;
        Gamma[0][5] = SOLVE_STEP_SQ_D4 * -q4;

        Gamma[1][0] = SOLVE_STEP_D2 *  q1;
        Gamma[1][1] = SOLVE_STEP_D2 * -q4;
        Gamma[1][2] = SOLVE_STEP_D2 *  q3;
        Gamma[1][3] = SOLVE_STEP_SQ_D4 *  q1;
        Gamma[1][4] = SOLVE_STEP_SQ_D4 * -q4;
        Gamma[1][5] = SOLVE_STEP_SQ_D4 *  q3;

        Gamma[2][0] = SOLVE_STEP_D2 *  q4;
        Gamma[2][1] = SOLVE_STEP_D2 *  q1;
        Gamma[2][2] = SOLVE_STEP_D2 * -q2;
        Gamma[2][3] = SOLVE_STEP_SQ_D4 *  q4;
        Gamma[2][4] = SOLVE_STEP_SQ_D4 *  q1;
        Gamma[2][5] = SOLVE_STEP_SQ_D4 * -q2;

        Gamma[3][0] = SOLVE_STEP_D2 * -q3;
        Gamma[3][1] = SOLVE_STEP_D2 *  q2;
        Gamma[3][2] = SOLVE_STEP_D2 *  q1;
        Gamma[3][3] = SOLVE_STEP_SQ_D4 * -q3;
        Gamma[3][4] = SOLVE_STEP_SQ_D4 *  q2;
        Gamma[3][5] = SOLVE_STEP_SQ_D4 *  q1;

        Gamma[4][0] = 0.0;
        Gamma[4][1] = 0.0;
        Gamma[4][2] = 0.0;
        Gamma[4][3] = SOLVE_STEP;
        Gamma[4][4] = 0.0;
        Gamma[4][5] = 0.0;

        Gamma[5][0] = 0.0;
        Gamma[5][1] = 0.0;
        Gamma[5][2] = 0.0;
        Gamma[5][3] = 0.0;
        Gamma[5][4] = SOLVE_STEP;
        Gamma[5][5] = 0.0;

        Gamma[6][0] = 0.0;
        Gamma[6][1] = 0.0;
        Gamma[6][2] = 0.0;
        Gamma[6][3] = 0.0;
        Gamma[6][4] = 0.0;
        Gamma[6][5] = SOLVE_STEP;

        Q[0] = filter->Q[0];
        Q[1] = filter->Q[1];
        Q[2] = filter->Q[2];
        if(update == 1)
        {
            Q[3] = filter->Q[3];
            Q[4] = filter->Q[4];
            Q[5] = filter->Q[5];
        }
        else
        {
            Q[3] = 0.0;
            Q[4] = 0.0;
            Q[5] = 0.0;
        }

        matrixmul((double*)P2, (double*)Phi, (double*)filter->P, 7, 7, 7); //P2 = Phi*P
        matrixmultran((double*)P1, (double*)P2, (double*)Phi, 7, 7, 7); //P1 = P2*Phi' = Phi*P*Phi'
        matrixmuldiag((double*)P3, (double*)Gamma, Q, 7, 6); //P3 = Gamma*Q
        matrixmultran((double*)P2, (double*)P3, (double*)Gamma, 7, 6, 7); //P2 = P3*Gamma' = Gamma*Q*Gamma'
        if(update == 1)
            matrixadd((double*)P2, (double*)P1, (double*)P2, 7, 7); //P2 = P1+P2 = Phi*P*Phi'+Gamma*Q*Gamma'
        else
            matrixadd((double*)filter->P, (double*)P1, (double*)P2, 7, 7); //P = P1+P2 = Phi*P*Phi'+Gamma*Q*Gamma'

        /* Measure update */
        if(update == 1)
        {
            u = 2.0 * (q2*q3 + q1*q4);
            v = q1*q1 + q2*q2 - q3*q3 - q4*q4;
            w = 2.0 / (u*u + v*v);

            dgx = imu->acc[0]/g + 2*(q2*q4 - q1*q3);
            dgy = imu->acc[1]/g + 2*(q3*q4 + q1*q2);
            dgz = imu->acc[2]/g + (q1*q1 - q2*q2 - q3*q3 + q4*q4);
            dpsi = angle[0] - atan2(u, v);
            if(dpsi > PI)
                dpsi -= PI_M2;
            else if(dpsi < -PI)
                dpsi += PI_M2;

            H[0][0] = 2.0 *  q3;
            H[0][1] = 2.0 * -q4;
            H[0][2] = 2.0 *  q1;
            H[0][3] = 2.0 * -q2;
            H[0][4] = 0.0;
            H[0][5] = 0.0;
            H[0][6] = 0.0;

            H[1][0] = 2.0 * -q2;
            H[1][1] = 2.0 * -q1;
            H[1][2] = 2.0 * -q4;
            H[1][3] = 2.0 * -q3;
            H[1][4] = 0.0;
            H[1][5] = 0.0;
            H[1][6] = 0.0;

            H[2][0] = 2.0 * -q1;
            H[2][1] = 2.0 *  q2;
            H[2][2] = 2.0 *  q3;
            H[2][3] = 2.0 * -q4;
            H[2][4] = 0.0;
            H[2][5] = 0.0;
            H[2][6] = 0.0;

            H[3][0] = w * (q4*v - q1*u);
            H[3][1] = w * (q3*v - q2*u);
            H[3][2] = w * (q2*v + q3*u);
            H[3][3] = w * (q1*v + q4*u);
            H[3][4] = 0.0;
            H[3][5] = 0.0;
            H[3][6] = 0.0;

            matrixmultran((double*)PH1, (double*)P2, (double*)H, 7, 7, 4); //PH1 = P*H'
            matrixmul((double*)PH2, (double*)H, (double*)PH1, 4, 7, 4); //PH2 = H*PH1 = H*P*H'
            PH2[0][0] += filter->R[0];
            PH2[1][1] += filter->R[1];
            PH2[2][2] += filter->R[2];
            PH2[3][3] += filter->R[3]; //PH2 = H*P*H'+R
            matrixinv((double*)PH3, (double*)PH2, 4); //PH3 = PH2^-1 = (H*P*H'+R)^-1
            matrixmul((double*)K, (double*)PH1, (double*)PH3, 7, 4, 4); //K = PH1*PH3 = P*H'/(H*P*H'+R)
            matrixmul((double*)P1, (double*)K, (double*)H, 7, 4, 7); //P1 = K*H
            matrixmulconst((double*)P1, (double*)P1, -1.0, 7, 7); //P1 = -K*H
            P1[0][0] += 1.0;
            P1[1][1] += 1.0;
            P1[2][2] += 1.0;
            P1[3][3] += 1.0;
            P1[4][4] += 1.0;
            P1[5][5] += 1.0;
            P1[6][6] += 1.0; //P1 = I-K*H
            matrixmul((double*)filter->P, (double*)P1, (double*)P2, 7, 7, 7); //P = (I-K*H)*P

            q1           += K[0][0]*dgx + K[0][1]*dgy + K[0][2]*dgz + K[0][3]*dpsi;
            q2           += K[1][0]*dgx + K[1][1]*dgy + K[1][2]*dgz + K[1][3]*dpsi;
            q3           += K[2][0]*dgx + K[2][1]*dgy + K[2][2]*dgz + K[2][3]*dpsi;
            q4           += K[3][0]*dgx + K[3][1]*dgy + K[3][2]*dgz + K[3][3]*dpsi;
            filter->X[4] += K[4][0]*dgx + K[4][1]*dgy + K[4][2]*dgz + K[4][3]*dpsi;
            filter->X[5] += K[5][0]*dgx + K[5][1]*dgy + K[5][2]*dgz + K[5][3]*dpsi;
            filter->X[6] += K[6][0]*dgx + K[6][1]*dgy + K[6][2]*dgz + K[6][3]*dpsi;
        }

        /* Diagonalization */
        if(filter->cnt == 500)
        {
            filter->cnt = 0;
			matrixsym((double*)filter->P, 7);
        }
        /*--------------------------------EKF--------------------------------*/

        /* Quaternion normalization */
        q = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
        filter->X[0] = q1 / q;
        filter->X[1] = q2 / q;
        filter->X[2] = q3 / q;
        filter->X[3] = q4 / q;

        /* Transform quaternion to angle */
        quat2angle(angle, filter->X);
        iner->att[0] = (float)(angle[0] * R2D);
        iner->att[1] = (float)(angle[1] * R2D);
        iner->att[2] = (float)(angle[2] * R2D);

        /* Save omega */
        w0[0] = w2[0];
        w0[1] = w2[1];
        w0[2] = w2[2];
    }
    else if(filter->flag == 1)
    {
        filter->flag = 2;
        /* Save omega */
        w1[0] = (double)imu->gyro[0] * D2R - filter->X[4];
        w1[1] = (double)imu->gyro[1] * D2R - filter->X[5];
        w1[2] = (double)imu->gyro[2] * D2R - filter->X[6];
    }
    else
    {
        filter->flag = 1;
        filter->cnt = 0;
        /* Save omega */
        w0[0] = (double)imu->gyro[0] * D2R;
        w0[1] = (double)imu->gyro[1] * D2R;
        w0[2] = (double)imu->gyro[2] * D2R;
        /* Initialize state variables */
        accmag2att(imu->acc, imu->mag, &angle[0], &angle[1], &angle[2]);
        angle2quat(filter->X, angle);
        filter->X[4] = 0.0;
        filter->X[5] = 0.0;
        filter->X[6] = 0.0;
        /* Initialize P */
        memset(filter->P, 0, sizeof(filter->P));
        filter->P[0][0] = (double)para[0];
        filter->P[1][1] = (double)para[1];
        filter->P[2][2] = (double)para[2];
        filter->P[3][3] = (double)para[3];
        filter->P[4][4] = (double)para[4];
        filter->P[5][5] = (double)para[5];
        filter->P[6][6] = (double)para[6];
        /* Initialize Q */
        filter->Q[0] = (double)para[7];
        filter->Q[1] = (double)para[8];
        filter->Q[2] = (double)para[9];
        filter->Q[3] = (double)para[10];
        filter->Q[4] = (double)para[11];
        filter->Q[5] = (double)para[12];
        /* Initialize R */
        filter->R[0] = (double)para[13];
        filter->R[1] = (double)para[14];
        filter->R[2] = (double)para[15];
        filter->R[3] = (double)para[16];
    }
}
