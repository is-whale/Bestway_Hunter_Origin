下面列举了导航包下config/目录下的 base_local_planner_params. yaml参数以及设定的值, 机器人在这样的设定下运行得相当好

1.base_local planner_params yaml 　　
controller_ frequency:3.0	-每多少秒我们需要更新一次路径规划?把这个值设得太高会使性能不足的CPU过载。对于一台普通的计算机来说,设定为3到5就可以运行得相当不错
max_vel_x:0.3	-机器人的最大线速度,单位是米/秒。对于室内机器人来说0.5就已经很快了,所以保守地选择设为0.3。
min vel x:0.05	-机器人的最小线速度
max rotation vel:1.0	-机器人的最大旋转速度,单位是弧度/秒。不要把这个值设得太高,不然机器人会错过它的目标方向
min_in_place_vel_theta:0.5	-机器人的最小原地旋转速度,单位是弧度/秒
escape_vel:-0.1		-机器人逃离时的速度,单位是米/秒。请注意这个值必须设为负数,这样机器人才能反向
mx:2.5		-在x方向上的最大线加速度
acc_lim_x:2.5	-在x方向上的最大线加速度
acc_lim_y:0.0	-在y方向上的最大线加速度。对于差速驱动(非完整驱动)机器人我们设为0,这样机器人就只能在x方向上线性运动(和旋转)了
acc_lim_theta:3.2	-最大角加速度。
holonomic_robot:false	-除非你有一个全方向驱动机器人,否则一律设为alse
yaw.goal_tolerane:0.1	-至多距离目标方向的误差(单位为弧度)是多少?把这个值设得太小的话可能会导致机器人在目标附近倒
xy_goal_tolerance:0.1	-至多距离目标位置的误差(单位为米)是多少?如果把值设得太小,你的机器人可能会不断地在目标位置附近作调整。注意:不要把最大误差设定得比地图的分辨率还小(下一部分会讲到),否则机器人会永不停息地在目标附近徘徊但水远到不了目标位置。
pdist_scale:0.8		-全局路径规划和到达目的位置之间的权重。这个参数比gist-seae大时,机器人更倾向于考虑全局路径规划
gdist_scale:0.4		-到达目的位置和全局路径规划之间的权重。这个参数比 pdist_scale大时,机器人会更考虑到达目标位置,而不管这段路在全局路径规划中是否是必须的
occdist_seale:0.1	-避开障碍物的权重
sim_time:	-规划时需要考虑未来多长时间(单位为秒)呢?这个参数和下一个参数(dwa)一起极大地影响机器人往目标移动的路径
dwa:true	-在模拟未来的轨迹时是否用动态窗口方法( Dynamie Window Approach)

2.costmap_common_params. yaml
这个文件中只有两个参数是需要你立刻为你的机器人调整的

robot radius:0.165	-对一个圆形的机器人来说,这是机器人的半径,单位是米;对一个非圆形的机器人来说,你可以用接下来说到的 footprint参数。这里用到的值是 Turtle的原始数据。
footprint:[[x0,y0],[xl,yl],x2,y2],[x3,y3],ect]	-在列表中的每一个坐标代表机器人的边上的一点,机器人的中心设为[0,0]。测量单位是米。这在机器人周长上的点要不按照顺时针排列,要不按照逆时针排列。
inflation_radius:0.3	-地图上的障碍物的半径,单位为米。如果你的机器人不能很好地通过窄门或其他狭窄的地方,则稍微减小这个值。相反的,如果机器人不断地撞到东西,则尝试增大这个值。

3.global_costmap_params. yaml
这个文件中有一些参数,你需要根据机器人CPU的计算能力和你的工作站与机器人之间的情况来实验得到。

global frame: /map	-对于全局代价地图,我们用map框架来作为 global框架
robot base frame: /base_ footprint	-这个通常不是/he-ink就是/ ase_footprint对于Turtle Bot应设为/ base_footprint
update_frequency:1.0		-根据传感器数据,全局地图更新的频率,单位为赫兹。这个数值越大,你的计算机的CPU负担会越重。特别对于全局地图,通常会设定一个相对较小、在1.0到5.0之间的值。
publish_frequency:0		-对于静态的全局地图来说,不需要不断发布。
statie_map:true		-这个参数和下一个参数通常会设为相反的值。全局地图通常是静态的,因此我们通常会把这个参数设为true
rolling window:true	-当我们把这个参数设为 false时,全局地图不会在机器人移动的时候更新。
transform_ tolerance.	-指定在tf树中框架之间的转换的最大延时,单位为秒。对于典型的工作站和机器人之间的无线网络来说,与1.0秒同一级别的都可以工作得很好

4.local_costmap_params yaml
有几个本地代价地图的参数需要考虑一下

global_ frame:/odom	-对于本地代价地图来说,我们使用 odometry框架来作为 global框架
robot_ base frame:/ base_footprint	-这个通常不是/ base link就是/ base_ footprint对于TurtleBot应设为/ base_footprint
update_frequency:3.0	-根据传感器数据,本地地图更新的频率,单位为次/秒。对于很慢的计算机,你可能需要减小这个值。
publish_ frequency:1.0	-我们想更新已经发布出去的本地地图,所以我们会把这个值设为非零。一秒一次应该足够了,除非你的机器人要移动得更快
static_map: false	-这个参数和下一个参数通常会设为相反的值。当本地地图需要根据传感器数据动态更新的时候,我们通常会把这个参数设为 false
rolling-window:true	-下面的几个参数定义了本地地图更新用的滑动窗口
width:6.0		-滑动地图的x维长度是多少米
height:6.0		-滑动地图的y维长度是多少米
resolution:0.01		-滑动地图的分辨率,单位为米。这个参数应该与YAML文件设置的地图分辨率匹配
ransform tolerance:1.0	-指定在tf树框架之间的转换,或可能会暂时中止的地图绘制过程中两者的最大延时,单位为秒。在一台速度较快、直连到机器人的计算机上,把这个值设定为1.0就能很好地工作了。但是在通过无线网络连接的较慢计算机上,这个延时容忍参数需要增大。
