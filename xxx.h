/**
 * @file xxx.h
 * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-08-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef XXX_H
#define XXX_H

/* Includes ------------------------------------------------------------------*/

//引用顺序从底层到顶层

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 枚举类型
 *
 */
enum Enum_Type
{
    Type_FIRST = 0,
    Type_SECOND,
};

/**
 * @brief 类
 *
 */
class Class_Type
{
public:
    void Init(int __A);

    int Get_A();

    void Set_A(int __A);

    void Callback();

protected:
    //初始化相关常量, 初始化后一成不变

    //常量, 一成不变, 可读

    //内部变量, 不对外接口

    //读变量, 只读不写

    //写变量, 只写不读

    //读写变量

    //内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

int Function_Name(int parameter);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
