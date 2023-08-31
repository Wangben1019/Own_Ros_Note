# ROS常用API
## 1. 初始化
```c++
/*
 * 作用：ROS初始化函数
 * 
 * 参数：1. argc ：封装实参的个数(n + 1)
 *      2. argv ：封装实参的数组，把参数和文件自身装入数组
 *      3. name ：节点的命名(唯一性)
 *      4. options ： 节点启动选项
 *      
 *      返回值：void
 * 使用：
 *      1. argc与argv的使用
 *          如果按照ROS中的特定格式传入实参，那么ROS就可以加以使用，比如设置全局参数，给节点重命名
 *          rosrun plumbing_apis demo01_apis_pub _length=2 
 *      2. options 的使用
 *          节点名称需要保证唯一，会导致一个问题：同一个节点不能重复启动。
 *          结果：ROS中当有重名的节点启动时，之前的节点会被关闭
 *          需求：特定场景下，需要一个节点多次启动且能正常运行
 *          解决：设置启动项：ros::init_options::AnonymousName
 *          ros::init(argc, argv, "erGouZi", ros::init_options::AnonymousName);
 *          每次启动在 "erGouZi" (用户自定义节点名称)后，添加随机未知数，保证同一个节点能够重新启动
 *          调用rosnode list可以查看节点名称
 * */
```
## 2. 话题与服务相关对象
**在roscpp中，话题和服务的相关对象都是由NodeHandle创建**
**NodeHandle有一个重要作用是可以用于设置命名空间**
### 发布方对象
```c++
// 创建节点句柄
ros::NodeHandle nh;
// 创建发布者对象
/*
 * nh.advertise<>();
 * 作用：创建发布者对象
 * 
 * 模板：被发布消息的对象
 * 
 * 参数：
 *      1. 话题名称
 *      2. 队列长度
 *      3. latch(可选) bool:如果设置为true，会保存发布方的最后一条消息，并且新的
 *          订阅对象连接到发布方时，发布方会将这条消息发送给订阅者
 * 
 * 使用：latch 设置为 true 的作用？
 *      以静态地图发布为例：方案1：以固定频率发送地图数据，但是效率低
 *      方案2：可以将地图发布对象的latch设置为true,并且发布方只发送一次数据，每当订阅者连接时
 *      就将地图数据发送给订阅者（只发送一次），这样就提高了数据发送效率
 * */
ros::Publisher pub = nh.advertise<std_msgs::String>("fang", 10, true);
```
### 回旋函数
略
### 时间
#### 1. 时刻
**获取时刻，或是设置指定时刻**
```c++
/*
 *  需求：演示时间相关操作（获取当前时刻 + 设置指定时刻）
 *  实现：
 *      1. 准备（头文件，节点初始化，NodeHandle创建）
 *      2. 获取当前时刻
 *      3. 设置当前时刻
 * */
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "hello_time");
    ros::NodeHandle nh;
    
    // 2. 获取当前时间
    // now 函数会将当前的时刻封装并返回
    // 当前时刻：now 被调用执行的那一刻
    // 参考系：1970年01月01日 00:00:00
    ros::Time right_now = ros::Time::now();
    // 将时间变成秒
    double d = right_now.toSec();
    ROS_INFO("当前时刻：%.2f", d);
    int i = right_now.sec;
    ROS_INFO("当前时刻：%d", i);
    // 3.设置制定时刻
    ros::Time t1(20, 312345678);
    ROS_INFO("t1 = %.2f", t1.toSec());
    ros::Time t2(100.35);
    ROS_INFO("t2 = %.2f", t2.toSec());
    
    return 0;
}

```
#### 持续时间
```c++
/*
 *  需求：程序执行中停顿五秒
 *  实现：
 *      1. 创建持续时间对象
 *      2. 休眠
 * 
 * */
    ros::init(argc, argv, "hello_time");
    ros::NodeHandle nh;
    ros::Duration du(4.5);
    du.sleep();

```
#### 持续时间与时刻运算
```c++
/*
 *  需求：已知程序开始运行的时刻和程序的运行时间，求运行结束的时刻
 *  实现：
 *      1. 获取开始执行的时刻
 *      2. 模拟运行时间
 *      3. 计算结束时刻 = 开始时刻 + 持续时间
 *      
 *  注意：
 *      1. 时刻与持续时间可以执行加减
 *      2. 持续时间之间也可以执行加减
 *      3. 时刻之间可以相减，不可以相加
 * */

// ros::Duration 持续时间
// ros::Time 时刻

// 1. 获取开始执行的时刻
ros::Time begin = ros::Time::now();
// 2. 模拟运行时间
ros::Duration du1(5);
// 3. 计算结束时刻 = 开始时刻 + 持续时间
ros::Time stop = begin + du1;
ROS_INFO("开始时刻:%.2f", begin.toSec());
ROS_INFO("结束时刻:%.2f", stop.toSec());

// 时刻与时刻运算
// ros::Time sum = begin + stop; // 不可以相加
ros::Duration du2 = begin - stop;
ROS_INFO("时刻相减：%.2f",du2.toSec());
// 持续时间与持续时间的运算
ros::Duration du3 = du1 + du2; // 0
ros::Duration du4 = du1 - du2; // 10
ROS_INFO("du1 + du2 = %.2f,du1 - du2 = %.2f", du3.toSec(), du4.toSec());
```
#### 设置运行频率
```c++
ros::Rate rate(1); // 设定频率 单位：Hz
while(true)
{
    rate.sleep();    
}
```
#### 定时器
```c++
/*
 *  需求：每隔一秒钟，在控制台输出一段文本。
 *  实现：
 *      1. 策略1：ros::Rate rate(1);
 *      2. 策略2：定时器
 * 
 * */
void cb(const ros::TimerEvent& event)
{
    ROS_INFO("--------------6---------------");
    ROS_INFO("函数被调用的时刻：%.2f", event.current_real.toSec());
}
/*
    ros::Timer createTimer( ros::Duration period,   // 时间间隔 --- 1s
                            const ros::TimerCallback &callback,     //回调函数 --- 封装业务
                            bool oneshot = false, // 是否是一次性，true为一次性
                            bool autostart = true) const  // 是否是自动启动
    */
ros::Timer timer = nh.createTimer(ros::Duration(1), cb);
ros::spin();

ros::Timer timer = nh.createTimer(ros::Duration(1), cb, true, false);
timer.start(); // 手动启动定时器

// 形参详解
```
* **注意：**
    * **创建：nh.createTimer()**
    * **参数1：时间回调**
    * **参数2：回调函数(时间时间 TimerEvent)**
    * **参数3：是否只执行一次**
    * **参数4：是否自动启动(当设置为false，需要手动调用 timer.start())**
    * **定时器启动后：ros::spin()**

### 其他函数
#### 节点生命周期
**在发布实现时，一般会循环发布消息，循环的判断条件一般由节点状态来控制，C++中可以通过 ros::ok() 来判断节点状态是否正常**
**导致节点退出的原因主要有如下几种:**
* **节点接收到了关闭信息，比如常用的 ctrl + c 快捷键就是关闭节点的信号；**
* **同名节点启动，导致现有节点退出；**
* **程序中的其他部分调用了节点关闭相关的API(C++中是ros::shutdown())**

#### 日志相关
* **DEBUG(调试):只在调试时使用，此类消息不会输出到控制台；** 
* **INFO(信息):标准消息，一般用于说明系统内正在执行的操作；**
* **WARN(警告):提醒一些异常情况，但程序仍然可以执行；**
* **ERROR(错误):提示错误信息，此类错误会影响程序运行；**
* **FATAL(严重错误):此类错误将阻止节点继续运行。**

