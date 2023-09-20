//
// Created by ShiF on 2023/9/12.
//
#include "ADRC.h"
#include "framework_headfile.hpp"
/** 日志
 * ADRC目前没有做限幅
 */
/** ADRC算法参数记录
 *
 *  调参顺序:TD -> NONLINER FEEDBACK <--> ESO（ ESO 和 NONLINER FEEDBACK 联调）
 *  使用方式：初始化后，要对参数进行标定 新建一个函数 ADRC_Param()
 *      第二轮考核，调试单电机参数记录
 *      ADRC_motor.r = 200;
        ADRC_motor.h = 0.05;
        ADRC_motor.delta = 0.5;
        ADRC_motor.b = 5;
        ADRC_motor.beta01 = 1;
        ADRC_motor.beta02 = 300;
        ADRC_motor.beta03 = 5;
        ADRC_motor.alpha1 = 0.5;
        ADRC_motor.alpha2 = 1.25;
        ADRC_motor.betac1 = 80;
        ADRC_motor.betac2 = 1;
 *
 */

/**
 * sgn 函数
 * @param x
 * @return
 */
float sgn(float x) {
    float sgn_out;
    if (x > 0) sgn_out = 1;
    else if (x < 0) sgn_out = -1;
    else sgn_out = 0;

    return sgn_out;
}

/**
 * fal 函数
 * @param e
 * @param alpha
 * @param delta
 * @return
 */
float fal(float e, float alpha, float delta) {
    float fal_out;
    if (fabsf(e) > delta) fal_out = powf(fabsf(e), alpha) * sgn(e);
    else fal_out = e / (powf(delta, 1 - alpha));
    return fal_out;
}

/**
 * fsg 函数
 * @param x
 * @param y
 * @return
 */
float fsg(float x, float y) {
    float fsg_out;
    fsg_out = (sgn(x + y) - sgn(x - y)) / 2;
    return fsg_out;
}

/**
 * fhan 函数
 * @param x1
 * @param x2
 * @param r
 * @param h
 * @return
 */
float fhan(float x1, float x2, float r, float h) {
    float d, a0, a1, a2, a, y, fhan_out;
    d = powf(h, 2) * r;
    a0 = h * x2;
    y = x1 + a0;
    a1 = sqrtf(d * (d + 8 * fabsf(y)));
    a2 = a0 + sgn(y) * (a1 - d) * 0.5;
    a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
    fhan_out = -r * (a / d) * fsg(y, d) - r * sgn(a) * (1 - fsg(a, d));
    return fhan_out;
}

/**
 * TD微分跟踪器
 * @param which
 */
void ADRC_TD(ADRC_t *which) {
    float fh;
    fh = fhan(which->x1 - which->target, which->x2, which->r, which->h);
    which->x1 += which->h * which->x2;
    which->x2 += which->h * fh;
//    usart_printf("%.2f,%.2f\r\n", which->x1, which->x2);
}

/**
 * ESO自干扰观测器
 * @param which
 */
void ADRC_ESO(ADRC_t *which) {
    float fe, fe1;
    which->epsilon = which->z1 - which->feedback;
    fe = fal(which->epsilon, 0.5, which->delta);
    fe1 = fal(which->epsilon, 0.25, which->delta);
    which->z1 += which->h * (which->z2 - which->beta01 * which->epsilon);
    which->z2 += which->h * (which->z3 - which->beta02 * fe + which->u * which->b);
    which->z3 += which->h * (fe1 * which->beta03 * (-1));
}

/**
 * 非线性反馈
 * @param which
 */
void ADRC_nonlinerfeedback(ADRC_t *which) {
    float e1, e2;
    e1 = which->x1 - which->z1;
    e2 = which->x2 - which->z2;
    which->u0 = which->betac1 * fal(e1, which->alpha1, which->delta)
                + which->betac2 * fal(e2, which->alpha2, which->delta);
    which->u = which->u0 - which->z3 / which->b;
}

/**
 * ADRC参数初始化
 * @param which
 */
void ADRCparamInit(ADRC_t *which) {
    which->x1 = 0;
    which->x2 = 0;
    which->z1 = 0;
    which->z2 = 0;
    which->z3 = 0;
    which->target = 0;
    which->feedback = 0;
    which->delta = 0;
    which->u0 = 0;
    which->u = 0;
    which->r = 0;
    which->h = 0;
    which->epsilon = 0;
    which->b = 0;
    which->beta01 = 0;
    which->beta02 = 0;
    which->beta03 = 0;
    which->betac1 = 0;
    which->betac2 = 0;
    which->alpha1 = 0;
    which->alpha2 = 0;
    which->betac1 = 0;
    which->betac2 = 0;
}

/**
 * ADRC执行过程
 * @param which
 * @param target    目标值
 * @param feedback  反馈值
 * @return 计算出来的输出值
 */
float ADRC_calc(ADRC_t *which, float target, float feedback) {
    which->target = target;
    which->feedback = feedback;
    ADRC_TD(which);
    ADRC_ESO(which);
    ADRC_nonlinerfeedback(which);
    return which->u;
}



