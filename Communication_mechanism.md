# ROS通信机制

## 话题通信（一发一收式）
### 话题通信理论模型
* **话题通信是基于发布订阅模式的：一个节点发布消息，另一个节点订阅该消息**
* **话题通信适用于不断更新的数据传输相关的应用场景：不断更新、少逻辑处理的传输场景**
* **话题通信涉及到的三个角色：ROS Master、Talker、Listener：ROS Master负责保管Talker和Listener的注册信息，并匹配话题相同的Talker和Listener**
#### 0. Talker注册：advertise("bar", foo::1234)
**Talker启动后，会通过RPC在ROS Master中注册自身信息，其中包含所发布的消息的话题名称，ROS Master会将节点的注册信息加入到注册表中**
#### 1. Listener注册：subscribe("bar") 
**Listener启动后，也会通过RPC在ROS Master中注册自身信息，包含需要订阅消息的话题名。ROS Master也会将节点的注册信息加入到注册表中**
#### 2. ROS Master实现信息匹配：{foo:1234}
**ROS Master会根据注册表中的信息匹配Talker和Listener，并通过RPC向Listener发送Talker的RPC地址信息**
#### 3. Listener向Talker发送请求：connect("scan", TCP)
**Listener根据收到的RPC地址，通过RPC向Talker发送连接请求，传输订阅的话题名称、消息类型以及通信协议(TCP/UDP)**
#### 4. Talker确认请求：TCP server:foo:2345
**Talker接收到Listener的请求后，也是通过RPC向Talker发送连接请求，传输订阅的话题名称、消息类型以及通信协议(TCP/UDP)**
#### 5. Listener与Talker建立连接：connect(foo:2345)
**Listener根据步骤4返回的消息使用TCP与Talker建立网络连接**
#### 6. Talker向Listener发送消息：data massages
**建立连接后，Talker开始向LIstener发布消息**

#### 注意
**1. Talker与Listener启动五先后顺序要求**
**2. Talker与Listener都可以有多个**
**3. Talker与Listener建立连接后，不再需要ROS Master。即：即使关闭ROS Master，Talker与Listener照常通信**

### 话题通信基本操作(C++)
#### 发布方常用函数
```c++
#include "std_msgs/String.h"    // 添加普通文本类型的消息
#include <sstream>

setlocale(LC_ALL, "");  // 设置编码，让终端输出中文
ros::init(argc, argv, "talker") //参数3：节点名称，唯一存在
ros::NodeHandle nh; // 创建节点句柄

// 实例化发布者对象
//泛型：发布的消息类型
//参数1：要发布的话题
//参数2：队列中最大的保存的消息数，如果超出阈值，先销毁早到的数据
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

//被发送的数据
std_msgs::String msg;
//msg.data = "hello";
std::string msg_front = "hello!";
int count = 0;

//一秒延时(发送周期)
ros::Rate r(1);
/* 补充3:
 *订阅时，第一条数据丢失
 *原因: 发送第一条数据时， publisher 还未在 roscore 注册完毕
 *解决: 注册后，加入休眠 ros::Duration(3.0).sleep(); 延迟第一条数据的发送 
 */
ros::Duration(3).sleep();
//节点不死
while(ros::ok())
{
    //使用stringstream拼接字符串(添加头文件sstream)
    std::stringstream ss;
    ss << msg_front << count;
    msg.data = ss.str();
    
    //发布消息
    pub.publish(msg);
    r.sleep();
    count++;
    //暂无应用，用在有回调的循环中，非循环用spin
    ros::spinOnce();
}
```

#### 订阅方常用函数
```c++
#include "ros/ros.h"
#include "std_msgs/String.h"
//4.实例化 订阅者 对象
    // 参数1 2与发布方相似，参数3为回调函数入口地址
    // 创建话题函数模板的类型可以不填，可以从回调函数的形参上自动推导
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter",10,doMsg);
    // 回调函数写法
    void doMsg(const std_msgs::String::ConstPtr &msg_p)
    {
    }
    // 设置循环调用回调函数
    ros::spin();
```

### 话题通信自定义msg（封装多个原生数据类型或其他特殊的类型）
**ROS中通过std_msgs封装了一些原生的数据类型：String、int32、int64、Char、Bool、Emply**
**但是这些数据一般只包含一个data字段，数据的单一意味着功能上的局限性、不能更好的传输复杂的数据**

**msgs只是简单的文本文件，每行具有字段类型和字段名称，可以使用的字段类型有：**
* **int8, int16, int32, int64(或者无符号类型: uint * )**
* **float32, float64**
* **string**
* **time, duration** **(日期时间)**
* **other msg files**
* **variable-length array[] and fixed-length array[C]** **(可变长度数组[]和固定长度数组[C])**
**ROS中还有一种特殊类型：Header，标头包含时间戳和ROS中常用的坐标帧信息。会经常看到msg文件的第一行具有Header标头。**

#### 自定义msg文件
**在功能包下新建msg文件夹目录，添加文件Person.msg**
```msg
string name
uint16 age
float64 height
```
#### 编辑配置文件
**package.xml中添加编译依赖与执行依赖**
```xml
  <build_depend>message_generation</build_depend>  <!--编译依赖-->
  <exec_depend>message_runtime</exec_depend>       <!--执行依赖-->
```

**CMakeLists.txt编辑msg相关设置**
````cmake
## 在find_package中添加message_generation,需要加入 message_generation,必须有 std_msgs
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation # 添加此项
)

## 配置msg源文件
add_message_files(
        FILES
        Person.msg # 添加此项
)

# 生成消息时依赖于 std_msgs
generate_messages(
        DEPENDENCIES
        std_msgs # 添加此项
)

#执行时依赖
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES demo02_talker_listener
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime    # 将整条语句取消注释后添加message_runtime
        #  DEPENDS system_lib
)

# 添加此项 第一个参数为可执行文件
# 此项目的是防止编译生成工程在编译自定义msg之前造成报错
add_dependencies(demo03_pub_person ${PROJECT_NAME}_generate_messages_cpp)
````

#### 编译
**编译后会在工作空间目录下的devel文件夹中(开发空间)的include文件夹中生成自定义的对应功能包的同名文件夹，文件夹下有对应的头文件**

### 话题通信自定义msg调用
#### 0. vscode配置
**为了方便代码提示以及避免误抛异常，需要先配置 vscode，将前面生成的 head 文件路径配置进 c_cpp_properties.json 的 includepath属性**
```json
"/xxx/yyy workspace/devel/include/**"
// demo03_ws 为工作空间  devel 为开发空间 ** 为包含include 文件夹下的所有头文件
"/home/wang/study/ROS_Study/demo03_ws/devel/include/**"
```

#### 发布方使用部分要点_
```c++
// 调用时需要包含头文件，plumbing_pub_sub是编译生成的功能包同名文件夹
#include <plumbing_pub_sub/Person.h>

ros::NodeHandle nh;

ros::Publisher pub = nh.advertise<demo02_talker_listener::Person>("chatter_person",1000);
// 组织被发布的消息，编写发布逻辑并发布消息
demo02_talker_listener::Person p;
p.name = "sunwukong";
p.age = 2000;
p.height = 1.45;

pub.publish(p);
```

#### 订阅方使用部分要点
```c++
void doPerson(const demo02_talker_listener::Person::ConstPtr& person_p){
    ROS_INFO("订阅的人信息:%s, %d, %.2f", person_p->name.c_str(), person_p->age, person_p->height);
}
#include "demo02_talker_listener/Person.h"
ros::Subscriber sub = nh.subscribe<demo02_talker_listener::Person>("chatter_person",10,doPerson);
```

## 服务通信(一问一答式)

### 服务通信理论模型
**服务通信是基于请求响应模式的，是一种应答机制：一个节点A向另一个节点B发送请求，B接受处理请求并产生结果返回给A**
**用作偶然的，对实时性有要求的、有一定逻辑处理需求的数据传输场景**
**服务通信涉及到三个角色：ROS Master(管理者)，Server(服务端)，Client(客户端)**
**ROS Master负责保管Server和Client注册的信息，并匹配到话题相同的Server与Client，帮助Server与Client建立连接，建立连接后，Client发送请求信息，Server返回响应信息**

#### 0.Server注册：advertise Service("bar", foo:1234)
**Server启动后，会通过RPC在ROS Master中注册自身信息，其中包含提供的服务的名称。ROS Master会将节点的注册信息加入到注册表中**

#### 1.Client注册：lookupService("bar")
**Client启动后，也会通过RPC在ROS Master中注册自身信息，包含需要请求的服务的名称。ROS Master会将节点的注册信息加入到注册表中**

#### 2.ROS Master实现信息匹配：{foo:3456}
**ROS Master会根据注册表中的信息匹配Server和Client，并通过RPC向Client发送Server的TCP地址信息**

#### 3.Client发送请求：request data (args)
**Client根据步骤2响应的信息，使用TCP与Server建立网络连接，并发送请求数据**

#### 4.Server发送响应：reply data
**Server接受、解析请求的数据、并产生响应结果返回给Client**

#### 注意
* **客户端请求被处理时，需要保证服务器已经启动**
* **服务端和客户端都可以存在多个**

### 服务通信自定义srv

#### 1.定义srv文件
**服务通信中，数据分成两部分，请求与响应，在srv文件中请求和响应使用``---``分隔**
**功能包下新建srv目录，添加xxx.srv文件，内容：**
```text
# 客户端请求发送的两个数字
int32 num1
int32 num2
---
# 服务器响应发送的数据
int32 sum
```

#### 2.编辑配置文件
**package.xml中添加编译依赖与执行依赖** **与自定义msg配置相同**
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
**CMakeLists.txt编辑 srv 相关配置** **与自定义msg配置相同**
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
# 需要加入 message_generation,必须有 std_msgs
add_service_files(
        FILES
        AddInts.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

#执行时依赖 官网没有在 catkin_package 中配置 message_runtime,经测试配置也可以
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES demo02_talker_listener
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime    # 将整条语句取消注释后添加message_runtime
        #  DEPENDS system_lib
)
# 添加此项 第一个参数为可执行文件
# 此项目的是防止编译生成工程在编译自定义msg之前造成报错
add_dependencies(demo03_pub_person ${PROJECT_NAME}_gencpp)
```
#### 编译
**编译后会在工作空间目录下的devel文件夹中(开发空间)的include文件夹中生成自定义的对应功能包的同名文件夹，文件夹下有对应的头文件**

### 话题通信自定义msg调用
#### 0. vscode配置
**为了方便代码提示以及避免误抛异常，需要先配置 vscode，将前面生成的 head 文件路径配置进 c_cpp_properties.json 的 includepath属性**
```json
"/xxx/yyy workspace/devel/include/**"
// demo03_ws 为工作空间  devel 为开发空间 ** 为包含include 文件夹下的所有头文件
"/home/wang/study/ROS_Study/demo03_ws/devel/include/**"
```
#### 1.服务端
```c++
#include <ros/ros.h>
#include <plumbing_server_client/Addints.h>

// 4.创建 服务 对象
ros::ServiceServer server = nh.advertiseService("AddInts",doReq);

ros::spin();

// 回调函数：参数1为客户端发来服务端的请求数据  参数2为服务端处理之后发送到客户端的数据
// bool 返回值由于标志是否处理成功 
bool doReq(plumbing_server_client::AddInts::Request& req,
          plumbing_server_client::AddInts::Response& resp){
    int num1 = req.num1;
    int num2 = req.num2;
    
    ROS_INFO("服务器接收到的请求数据为:num1 = %d, num2 = %d",num1, num2);
    
    //逻辑处理
    if (num1 < 0 || num2 < 0)
    {
        ROS_ERROR("提交的数据异常:数据不可以为负数");
        return false;
    }

    //如果没有异常，那么相加并将结果赋值给 resp
    resp.sum = num1 + num2;
    return true;//返回true则为服务器响应（猜的）
}
}
```

#### 2.客户端
```c++
#include <ros/ros.h>
#include <plumbing_server_client/Addints.h>

    // 4.创建 客户端 对象
    //nh.serviceClient<>() 返回值为ros::ServiceClient
    ros::ServiceClient client = nh.serviceClient<plumbing_server_client::AddInts>("AddInts");
    // 等待服务启动成功
    // 在客户端发送请求前添加:client.waitForExistence();
    // 或:ros::service::waitForService("AddInts");
    // 这是一个阻塞式函数，只有服务启动成功后才会继续执行
    //方式1
    ros::service::waitForService("AddInts");
    //方式2
    client.waitForExistence();
    
    plumbing_server_client::AddInts ai;
    //组织请求
    ai.request.num1 = 100;
    ai.request.num2 = 200;
    //处理响应
    bool flag = client.call(ai);
    //true 处理成功
    //响应结果
    ai.response.sum();
    
```

#### 3.客户端优化（使用终端动态提交请求数据）
```c++
/*
 * 实现参数的动态提交
 *  1. 格式：rosrun xxxx xxxx num1 num2
 *  2. 节点执行时，需要获取命令中的参数，并组织近request
 * */

int main(int argc, char *argv[])
{
    // 当从命令行传入两个数据num1，num2时，argc = 3
    // 程序本身有一个参数，程序本身的入口
    if(argc != 3)
    {
        ROS_ERROR("请提交两个整数");
        return 1;
    }
    // 5.组织请求数据
    // argv[0]：函数本身入口
    // argv[1]：num1
    // argv[2]：num2
    demo03_server_client::AddInts ai;
    ai.request.num1 = atoi(argv[1]); 
    ai.request.num2 = atoi(argv[2]);
}
```

## 参数服务器(实现多个节点之间数据共享)
**以共享的方式实现不同节点之间数据交互的通信模式**
**存储一些多节点共享的数据，类似于全局变量。**
### 参数服务器的理论模型
**参数服务器涉及到三个角色：ROS Master、Talker、Listener**
**ROS Master 作为一个公共容器保存参数，Talker 可以向容器中设置参数，Listener 可以获取参数。**

#### 1.Talker设置参数：setParam("foo", 1)
**Talker通过RPC向参数服务器发送参数（包括参数名与参数值），ROS Master将参数保存到参数列表中**

#### 2.Listener获取参数：getParam("foo")
**Listener通过RPC向参数服务器发送参数查找请求，请求中包含要查找的参数名**

#### 3.ROS Master 向 Listener发送参数值：{foo:1}
**ROS Master根据步骤2请求提供的参数名查找参数值，并将查询结果通过RPC发送给Listener**

#### 注意
**参数服务器不是为高性能而设计的，因此最好用于存储静态的非二进制的简单数据**

```text
参数可使用数据类型:

* 32-bit integers

* booleans

* strings

* doubles

* iso8601 dates

* lists

* base64-encoded binary data

* 字典
```

### 参数操作（参数的增删改查）
**在 C++ 中实现参数服务器数据的增删改查，可以通过两套 API 实现:**
* **ros::NodeHandle**
* **ros::param**

#### 1.参数服务器新增(修改)参数
```c++
/*
    参数服务器操作之新增与修改(二者API一样)_C++实现:
    相同的键，不同的值就可以实现更改
    在 roscpp 中提供了两套 API 实现参数操作
    ros::NodeHandle
        setParam("键",值)
    ros::param
        set("键","值")
*/
ros::NodeHandle nh;
nh.setParam("nh_int",10); //整型
nh.setParam("nh_double",3.14); //浮点型
nh.setParam("nh_bool",true); //bool
nh.setParam("nh_string","hello NodeHandle"); //字符串
nh.setParam("nh_vector",stus); // vector
//修改演示(相同的键，不同的值)
nh.setParam("nh_int",10000);

//param--------------------------------------------------------
ros::param::set("param_int",20);
ros::param::set("param_double",3.14);
ros::param::set("param_string","Hello Param");
ros::param::set("param_bool",false);
ros::param::set("param_vector",stus);
//修改演示(相同的键，不同的值)
ros::param::set("param_int",20000);
```
#### 2.参数服务器获取参数
```c++
/*
 * ros::NodeHandle
 *  1.param(键,默认值) 
 *      存在，返回对应结果，否则返回默认值 
 *  2.getParam(键,存储结果的变量)   
 *      存在,返回 true,且将值赋值给参数2, 若果键不存在，那么返回值为 false，且不为参数2赋值
 *  3.getParamCached键,存储结果的变量)--提高变量获取效率    
 *      存在,返回 true,且将值赋值给参数2, 若果键不存在，那么返回值为 false，且不为参数2赋值
 *  4.getParamNames(std::vector<std::string>)  
 *      获取所有的键,并存储在参数 vector 中
 *  5.hasParam(键)
 *      是否包含某个键，存在返回 true，否则返回 false
 *  6.searchParam(参数1，参数2)  
 *      搜索键，参数1是被搜索的键，参数2存储搜索结果的变量
 *      
 *   ros::param ----- 与 NodeHandle 类似   
 * */
/*  ros::NodeHandle nh  */
ros::NodeHandle nh;
/* 1. */
// 如果键存在，res1赋值为键对应的值，否则赋值100
int res1 = nh.param("nh_int",100);

/* 2. 3. */
int nh_int_value;
nh.getParam("nh_int",nh_int_value); //getParamCached  两者用法相似

/* 4. */
std::vector<std::string> param_names1;
nh.getParamNames(param_names1);
// 输出数据
for (auto &&name : param_names1)
{
    ROS_INFO("名称解析name = %s",name.c_str());
}

/* 6. */
std::string key;
nh.searchParam("nh_int",key);

/*  ros::param  */  /* 与 ros::NodeHandle 相似 */
```

#### 3.参数服务器删除参数
```c++
/*
 * ros::NodeHandle
 *  deleteParam("键")
 *     根据键删除参数，删除成功，返回 true，否则(参数不存在)，返回 false
 * ros::param
 *  del("键")
 *      根据键删除参数，删除成功，返回 true，否则(参数不存在)，返回 false
 * */
```

## ROS通信机制中常用命令
* **rosnode：操作节点**
* **rostopic：操作话题**
* **rosservice：操作服务**
* **rosmsg：操作msg消息**
* **rossrv：操作srv消息**
* **rosparam：操作参数**

### rosnode
**rosnode 是用于获取节点信息的命令**
```text
rosnode ping    测试到节点的连接状态
rosnode list    列出活动节点
rosnode info    打印节点信息
rosnode machine    列出指定设备上节点
rosnode kill    杀死某个节点
rosnode cleanup    清除不可连接的节点
```
* **rosnode ping**
    **测试到节点的连接状态**
* **rosnode list**
    **列出活动节点**
* **rosnode info**
    **打印节点信息**
* **rosnode machine**
    **列出指定设备上的节点**
* **rosnode kill**
    **杀死某个节点**
* **rosnode cleanup**
    **清除无用节点，启动乌龟节点，然后 ctrl + c 关闭，该节点并没被彻底清除，可以使用 cleanup 清除节点**

### rostopic
**rostopic包含rostopic命令行工具，用于显示有关ROS 主题的调试信息，包括发布者，订阅者，发布频率和ROS消息。**
**它还包含一个实验性Python库，用于动态获取有关主题的信息并与之交互。**
```text
rostopic bw     显示主题使用的带宽
rostopic delay  显示带有 header 的主题延迟
rostopic echo   打印消息到屏幕
rostopic find   根据类型查找主题
rostopic hz     显示主题的发布频率
rostopic info   显示主题相关信息
rostopic list   显示所有活动状态下的主题
rostopic pub    将数据发布到主题
rostopic type   打印主题类型
```

* **rostopic list(-v)**
    **直接调用即可，控制台将打印当前运行状态下的主题名称**
    **rostopic list -v：获取话题详情(比如列出：发布者和订阅者个数)**
* **rostopic pub**
    **可以直接调用命令向订阅者发布消息**
    **为roboware自动生成的 发布/订阅 模型案例中的 订阅者 发布一条字符串**
```text
rostopic pub /话题名称 消息类型 消息内容
rostopic pub liaoTian plumbing_pub_sub/Person "name:huluwa age:8 height:0.8"
# 按两下Tab即可补全消息内容部分，直接修改数据即可

rostopic pub -r 10 liaoTian plumbing_pub_sub/Person "name:huluwa age:8 height:0.8"
# 每秒发布十次
```
* **rostopic echo liaoTian**
    **打印话题为"liaoTian"的消息到屏幕**
    **由于自定义消息在工作空间下，所以需要在工作空间下打开终端才能运行命令，否则报错**
* **rostopic info liaoTian**
    **查看当前话题的信息**
* **rostopic type**
    **列出话题的消息类型**
* **rostopic find 消息类型**
    **根据消息类型查找话题**
* **rostopic delay**
    **列出消息头信息**
* **rostopic hz**
    **列出消息发布频率**
* **rostopic bw**
    **列出消息发布带宽**

### rosmsg
**rosmsg是用于显示有关 ROS消息类型的 信息的命令行工具。**
```text
rosmsg show    显示消息描述
rosmsg info    显示消息信息
rosmsg list    列出所有消息
rosmsg md5    显示 md5 加密后的消息
rosmsg package    显示某个功能包下的所有消息
rosmsg packages    列出包含消息的功能包

```

* **rosmsg list**
    **会列出当前 ROS 中的所有 msg**
* **rosmsg packages**
    **列出包含消息的所有包**
* **rosmsg package**
    **列出某个包下的所有msg**
```text
//rosmsg package 包名 
rosmsg package turtlesim
```
* **rosmsg show**
    **显示消息描述**
```text
//rosmsg show 消息名称
rosmsg show plumbing_pub_sub/Person
# 结果
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

### rosservice
**rosservice包含用于列出和查询ROSServices的rosservice命令行工具。**
```text
rosservice args 打印服务参数
rosservice call    使用提供的参数调用服务
rosservice find    按照服务类型查找服务
rosservice info    打印有关服务的信息
rosservice list    列出所有活动的服务
rosservice type    打印服务类型
rosservice uri    打印服务的 ROSRPC uri
```

* **rosservice list**
    **列出所有活动的 service**
```text
~ rosservice list
/addInts
...
```
* **rosservice args**
    **打印服务参数**
* **rosservice call addInts "num1:1 num2:2"**
    **调用服务：按两下Tab即可补全参数部分**
    **为小乌龟的案例生成一只新的乌龟**
```text
rosservice call /spawn "x: 1.0
y: 2.0
theta: 0.0
name: 'xxx'"
name: "xxx"

//生成一只叫 xxx 的乌龟

```
* **rosservice info addInts**
    **获取服务话题详情**
* **rosservice type addInts**
    **打印消息类型**
    **plumbing_server_client_AddInts**

### rossrv
**rossrv是用于显示有关ROS服务类型的信息的命令行工具，与 rosmsg 使用语法高度雷同。**
```text
rossrv show    显示服务消息详情
rossrv info    显示服务消息相关信息
rossrv list    列出所有服务信息
rossrv md5    显示 md5 加密后的服务消息
rossrv package    显示某个包下所有服务消息
rossrv packages    显示包含服务消息的所有包
```

* **rossrv list**
    **会列出当前ROS中的所有srv消息**
```text
# 管道筛选消息
rossrv list | grep -i addInts
```
* **rossrv info plumbing_server_client/AddInts**
    **返回自定义类型的详细信息**

### rosparam
**rosparam包含rosparam命令行工具，用于使用YAML编码文件在参数服务器上获取和设置ROS参数。**
```text
rosparam set    设置参数
rosparam get    获取参数
rosparam load    从外部文件加载参数
rosparam dump    将参数写出到外部文件
rosparam delete    删除参数
rosparam list    列出所有参数
```
* **rosparam list**
    **列出所有参数**
```text
rosparam list

//默认结果
/rosdistro
/roslaunch/uris/host_helloros_virtual_machine__42911
/rosversion
/run_id
```

* **rosparam set**
    **设置参数**
```text
rosparam set name huluwa

# 创建了一个名字是name内容是huluwa 的变量

//再次调用 rosparam list 结果
/name
/rosdistro
/roslaunch/uris/host_helloros_virtual_machine__42911
/rosversion
/run_id
```
* **rosparam get**
    **获取参数**
```text
rosparam get name

//结果
huluwa
```
* **rosparam delete**
    **删除参数**
```text
rosparam delete name

//结果
//去除了name
```
* **rosparam load**
    **从外部文件（事先准备好.yaml文件）加载参数**
```text
rosparam load xxx.yaml
```
* **rosparam dump**
    **将参数写出到外部文件**
```text
rosparam dump yyy.yaml
# 会生成一个yyy.yaml的文件
```

## 乌龟实操
### 实操01：话题发布
#### 获取话题 
```text
1. rostopic list 列出所有话题
2. rqt_graph    通过计算图查看话题
```
#### 获取参数类型
```text
1. rostopic info /turtle1/cmd_vel

    Type: geometry_msgs/Twist
    
    Publishers: 
     * /teleop_turtle (http://wangpc:43437/)
    
    Subscribers: 
     * /turtlesim (http://wangpc:42517/)
 
2. rostopic type /turtle1/cmd_vel
 
    geometry_msgs/Twist
```
#### 获取参数具体结构
```text
1. rosmsg show geometry_msgs/Twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
2. rosmsg info geometry_msgs/Twist
    geometry_msgs/Vector3 linear    # 线速度：前进或者是后退的速度    单位：m/s
      float64 x     # x：前进或者是后退的速度
      float64 y     # y：平移的速度
      float64 z     # z：垂直方向上上下移动的速度
    geometry_msgs/Vector3 angular   # 角速度：转弯的速度            单位：rad/s
      float64 x(Roll)：翻滚角
      float64 y(Ptich)：俯仰角
      float64 z(Yaw)：偏航角
# 以飞机模型构建坐标系：
# 贯穿机头机尾的为X轴，转动X轴会造成飞机顺时针或逆时针转动，则称此轴为翻滚
# 贯穿两端机翼的为Y轴，转动Y轴会造成飞机向上仰或向下低机头，则称此轴为俯仰
# 垂直方向贯穿身为Z轴，转动Z轴会造成飞机在水平方向左右转动，则称此轴为偏航
```
**弧度**：单位弧度定义为圆弧长度等于半径时的圆心角(arc length = radius)：即1s转过一圈角速度则为2π
#### 速度具体赋值
```text
1. rostopic echo /turtle1/cmd_vel
# 打印话题：/turtle1/cmd_vel的消息到终端
    linear: 
      x: 2.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
    ---
    linear: 
      x: -2.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
    ---
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: -2.0
    ---
# 总结：线速度只有x 角速度只有z
```
### 话题订阅
#### 编写.launch文件
```launch
<!-- 启动乌龟GUI与键盘控制节点 -->

<launch>
    <!-- 乌龟GUI -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen" />
    <!-- 键盘控制 -->
    <node pkg="turtlesim" type="turtle_teleop_key" name="key" output="screen" />
</launch>
```
#### 运行.launch文件
```text
roslaunch plumbing_test start_turtle.launch
```
#### 获取话题
```text
rostopic list
    /rosout
    /rosout_agg
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose   # pose:位姿
```
#### 获取参数类型
````text
rostopic info /turtle1/pose
    Type: turtlesim/Pose
    
    Publishers: 
     * /turtle1 (http://wangpc:37783/)
    
    Subscribers: None

````
#### 获取参数类型具体组成
```text
rosmsg info turtlesim/Pose 
    float32 x   # 坐标
    float32 y   # 坐标
    float32 theta   # 朝向
    float32 linear_velocity #线速度
    float32 angular_velocity    #角速度
```

### 服务调用
#### 服务名称与服务消息的获取
```text
# 话题获取：/spawn
rosservice list
    /clear
    /key/get_loggers
    /key/set_logger_level
    /kill
    /reset
    /rosout/get_loggers
    /rosout/set_logger_level
    /spawn  # 产卵：在GUI中产生新的小乌龟
    /turtle1/get_loggers
    /turtle1/set_logger_level
    /turtle1/set_pen
    /turtle1/teleport_absolute
    /turtle1/teleport_relative

```
```text
# 获取消息类型：turtlesim/Spawn
rosservice info /spawn
    Node: /turtle1
    URI: rosrpc://wangpc:33363
    Type: turtlesim/Spawn   # 数据类型
    Args: x y theta name    # 请求服务需要传入的参数
   
# 获取消息格式    
rossrv info turtlesim/Spawn # 数据类型具体格式
    float32 x
    float32 y
    float32 theta
    string name
    ---
    string name

```

### 参数服务器
```text
# 获取参数列表
rostopic list 
    /rosout
    /rosout_agg
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose
    
# 获取参数数据
rosparam get /turtlesim/background_r
69
rosparam get /turtlesim/background_b
255
rosparam get /turtlesim/background_g
86

# 修改参数
rosparam set /turtlesim/background_r 255
rosparam set /turtlesim/background_b 0
rosparam set /turtlesim/background_g 0
```