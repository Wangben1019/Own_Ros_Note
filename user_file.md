# ROS中的头文件与源文件

## 自定义头文件的调用
1. **在功能包下的 include/功能包名 目录下新建头文件**
```c++
#ifndef _HELLO_H
#define _HELLO_H

namespace hello_ns{

class HelloPub {

public:
    // 函数声明
    void run();
};

}

#endif
```
2. **在 VScode 中，为了后续包含头文件时不抛出异常，请配置 .vscode 下 c_cpp_properties.json 的 includepath属性**
```json
"/home/用户/工作空间/src/功能包/include/**"
```
3. **在可执行文件里调用**
```c++
// 包含头文件
#include "test_head/hello.h"

// 函数定义 (hello_ns命名空间)
namespace hello_ns {

void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
}

//函数调用 (hello_ns命名空间)
hello_ns::HelloPub helloPub;
helloPub.run();
```
4. **配置CMakeLists.txt文件**
**配置包含头文件**
```cmake
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
# （118行 - 121行）
# 未被修改的CMakeLists文件中  include 是被注释的  取消注释即可
# include_directories	向工程添加多个特定的头文件搜索路径
# 将/usr/include/myincludefolder 和 ./include 添加到头文件搜索路径
# include_directories(/usr/include/myincludefolder ./include)

# 本项目是讲与CMakeLists在同级目录下的include空间添加到头文件搜索路径，所以写法为：
# include_directories(include ${catkin_INCLUDE_DIRS})
```
**配置可执行文件方法不变**

## 自定义源文件调用
1. **在功能包下的 include/功能包名 目录下新建头文件**
```c++
#ifndef _HAHA_H
#define _HAHA_H

namespace hello_ns {

class My {

public:
    void run();

};

}

#endif
```
2. **在 VScode 中，为了后续包含头文件时不抛出异常，请配置 .vscode 下 c_cpp_properties.json 的 includepath属性**
```json
"/home/用户/工作空间/src/功能包/include/**"
```
3. **编写源文件**
```c++
#include "test_head_src/haha.h"
#include "ros/ros.h"

namespace hello_ns{

void My::run(){
    ROS_INFO("hello,head and src ...");
}

}
```
4. **在可执行文件中调用**
```c++
#include "test_head_src/haha.h"

hello_ns::My my;
my.run();
```
5. **配置CMakeLists文件**
**头文件与源文件配置**
```cmake
# 与自定义头文件调用相同，首先添加头文件搜索路径 （118行 - 121行）
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# 声明C++库 （127行 - 130行）
# 生成对象库文件，库文件名称为 head_src
# 参数2的include是与CMakeLists同级文件夹下的include文件夹
# ${PROJECT_NAME} 是功能包的名称，
add_library(head_src
        include/${PROJECT_NAME}/hello.h
        src/hello.cpp
        )

add_dependencies(head_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head_src
        ${catkin_LIBRARIES}
        )
```
**可执行文件配置**
```cmake
add_executable(use_head src/use_head.cpp)

add_dependencies(use_hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(use_hello
        head_src
        ${catkin_LIBRARIES}
        )
```