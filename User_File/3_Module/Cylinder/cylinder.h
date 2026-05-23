#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
class Cylinder{
public:
    Cylinder(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
    : GPIOx_(GPIOx), GPIO_Pin_(GPIO_Pin) {}
    void charge();
    void release();
private:
    GPIO_TypeDef *GPIOx_;
    uint16_t GPIO_Pin_;
};
#endif