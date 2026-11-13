#pragma once
#include <cstddef>
// 裸 ARM 模拟测试使用内存填充，不访问芯片外设。
extern "C" void *memset(void *p,int value,std::size_t size) {
  auto *out=static_cast<unsigned char *>(p);for(std::size_t i=0;i<size;++i)out[i]=value;return p;
}
// 裸 ARM 模拟测试使用非重叠内存复制。
extern "C" void *memcpy(void *p,const void *q,std::size_t size) {
  auto *out=static_cast<unsigned char *>(p);auto *in=static_cast<const unsigned char *>(q);
  for(std::size_t i=0;i<size;++i)out[i]=in[i];return p;
}
// 测试对象均在栈上，虚析构表所需的删除入口不释放外部设备或内存。
void operator delete(void *) noexcept {}
// 提供编译器生成的带大小删除入口；测试不创建堆对象。
void operator delete(void *,std::size_t) noexcept {}
#define CHECK(v) do {if(!(v))return __LINE__;}while(0)
