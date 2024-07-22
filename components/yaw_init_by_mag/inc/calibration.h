#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./../../psins/inc/psins.h"

/*
** 磁力计采用离线校准，采集球面数据后导入matlab
** 使用matlab提供的magcal函数得到矫正参数(b = 零点偏移) (A = 矫正矩阵)
** 使用方式参考官方手册：https://ww2.mathworks.cn/help/nav/ref/magcal.html?s_tid=doc_ta
** https://ww2.mathworks.cn/help/nav/ug/magnetometer-calibration.html?searchHighlight=magcal&s_tid=srchtitle_magcal_3
** 将离线校准所得结果更新到下边的 A b变量内即可。
*/

const CVect3 b(0.38971913, 0.64481938, 0.31692976);
const CMat3 A(0.96653795, 0.018643096, -0.14354038,
              0.018643096, 0.99324846, -0.0064900443,
              -0.14354038, -0.0064900443, 1.0633615);

#ifdef __cplusplus
}
#endif

#endif