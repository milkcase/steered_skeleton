#pragma once
#include <cstdint>
#include <cmath>
// 模拟 CMSIS 基础向量缩放，Mahony 的四元数和反馈逻辑仍运行原有源码。
inline void arm_scale_f32(const float *a,float scale,float *out,uint32_t n) {
  for(uint32_t i=0;i<n;++i)out[i]=a[i]*scale;
}
// 模拟 CMSIS 向量加法，将 n 项逐项相加。
inline void arm_add_f32(const float *a,const float *b,float *out,uint32_t n) {
  for(uint32_t i=0;i<n;++i)out[i]=a[i]+b[i];
}
// 计算单个四元数乘积，使用与真实 CMSIS 相同的 qw/qx/qy/qz 顺序。
inline void arm_quaternion_product_single_f32(const float *a,const float *b,float *out) {
  out[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
  out[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
  out[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
  out[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}
// 模拟 CMSIS atan2 接口，使未改动的 AHRS 基类能够链接。
inline int arm_atan2_f32(float y,float x,float *out) { *out=std::atan2(y,x);return 0; }
