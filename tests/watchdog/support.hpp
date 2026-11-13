#pragma once
#include <cstddef>
// 裸 ARM 测试的内存填充，不调用外设。
extern "C" void *memset(void *p,int value,std::size_t n) {
  auto *out=static_cast<unsigned char *>(p);for(std::size_t i=0;i<n;++i)out[i]=value;return p;
}
// 裸 ARM 测试的非重叠复制。
extern "C" void *memcpy(void *p,const void *q,std::size_t n) {
  auto *out=static_cast<unsigned char *>(p);auto *in=static_cast<const unsigned char *>(q);
  for(std::size_t i=0;i<n;++i)out[i]=in[i];return p;
}
#define CHECK(v) do {if(!(v))return __LINE__;}while(0)
