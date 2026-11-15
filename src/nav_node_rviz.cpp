#include <algorithm>
//reverse,把parent容器从倒序变成正序
#include <chrono>//计时器
#include <cmath>//算距离用
#include <functional>//bind打包函数用
#include <memory>//智能指针
#include <queue>//队列容器
#include <string>
#include <utility>//pair类型
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>//goal_pose类型，用来接收rviz2的信息
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>//这三个都是costmap的头文件：
                                             //第一个是判断格子可不可以走的权
                                             //第二个是cmap地图，包括原点，栅格等
                                             //第三个订阅/map,膨胀，TF
#include <nav_msgs/msg/path.hpp>//发path消息
#include <rclcpp/rclcpp.hpp>

class Node
{
   public:
   int y;
   int x;
   double f;
   double g;//已走过的路的代价
   bool operator<(const Node& other)const
   {
      return f>other.f;
   }
   //重载<，因为优先队列的排序逻辑是如果a<b为真，则把b放在a前面，也就是大顶堆，
   // 而我现在需要小顶堆（把最短路径排在前面）来找最短路，那么就是如果a<b为真，我判断其为假，按照a>b处理，把a放在队列前面
};
//heuristic函数用来判断我目前要走的点离终点还有多远
double heuristic(int x1,int y1,int x2,int y2)
{
    double DX = std::abs(x1 - x2);
    double DY = std::abs(y1 - y2);
    //记不住这个距离叫什么……反正原理就是把长方形切割成一个最大正方形和一个长方形，
    //关于为什么不用欧式距离：
    // 因为我写的逻辑是八方向移动，用这种计算方法效率更高,而且可以通过平移变换证明用欧氏距离是存在误差的
   return std::max(DX, DY) + (std::sqrt(2.0) - 1.0) * std::min(DX, DY);
}

class NavNode : public rclcpp::Node
{
   public:
   NavNode() : rclcpp::Node("nav_node")//给节点命名为nav_node
   {
      costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
          "global_costmap", this->get_namespace(), "");//只是建一个叫做“global_costmap"的节点，
                                                       //对应yaml中的"global_costmap"，从nav_node那里继承命名空间
      costmap_ros_->configure();//读yaml里面的参数，读plugins按照顺序建静态图和膨胀图，初始的图为250*250
      costmap_ros_->activate();//订阅/map，发送costmap给rivz看

      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("astar_path", 10);

      goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "goal_pose", 10,
              [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { goalCallback(msg); });
//lambda要求这个订阅者接收到消息后把msg值传递给goalCallback这个函数，
// 从而把rviz传来的目标从世界坐标转化为栅格坐标，并反转坐标系，运行一次A*寻路
      timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                       [this]() { tryInit(); });//买一个闹钟，一秒五次去看看有没有收到我想要的地图
   }

   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmapNode() const
   {
      return costmap_ros_;
   }//只是为了从private权限的区域浅拷贝出一个指向costmap_ros_的指针，相当于拿一张可以调用costmap_ros_的身份牌

   private:
   void tryInit()
   {
      nav2_costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
      //快递到了，拆快递：图片的长，宽，面积，一格多大，原点
      // 250*250 是 Costmap2DROS 的默认尺寸，说明 static_layer 还没把真实地图塞进来
      if (cm->getSizeInCellsX() == 0 || cm->getSizeInCellsX() == 250)
      {
         return;
      }
      //if是判断地图有没有传进来并且传的对不对
       // Costmap2DROS 在 configure() 之后会先用默认的 width/height(各 5m) 建一张图。
       // 按当前 resolution 0.02 算就是 250x250，这不是我要的地图。
       // 注意：改了 width、height、resolution 之后这个数要跟着改！！！
      ready_ = true;//拿到地图了！
      timer_->cancel();
      //把闹钟按掉
      costmap_ = cm;
      h = static_cast<int>(costmap_->getSizeInCellsY());
      w = static_cast<int>(costmap_->getSizeInCellsX());
      RCLCPP_INFO(this->get_logger(), "Map is OK!The hight is %dThe wight is %d", h, w);

      cmap.assign(h, std::vector<int>(w, 0));//把空地图开出来
      const unsigned char* master = costmap_->getCharMap();//为什么用指针不用数组：
                                                           //指针只占八个字节的空间，但是整个地图都传递进来需要多于1MB的空间
      for (int y = 0; y < h; y++)
      {
         for (int x = 0; x < w; x++)
         {
            unsigned char gray = master[(h - 1 - y) * w + x];//这一行是因为costmap地图和一般的地图的坐标系是反转的，图像的y指向下方，costmap的y指向上方
            if (gray >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
                gray != nav2_costmap_2d::NO_INFORMATION)
            {
               cmap[y][x] = 255;
            }
            else
            {
               cmap[y][x] = 0;
            }//那两个是写在头文件里面的常量，前者为253,后者为255,yaml里面的数据负责算出把253,255的线画在哪里
         }
      }
   }

   void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
   {
      nav2_costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
      const double res = cm->getResolution();
      const double ox  = cm->getOriginX();
      const double oy  = cm->getOriginY();//不写死这几个值保证在yaml里面的值发生变化后代码仍然成立
      if (!ready_)
      {
         RCLCPP_WARN(this->get_logger(), "地图还没就绪，等 costmap 收到 /map 再点");
         return;
      }
      const int gx = static_cast<int>(std::floor((msg->pose.position.x - ox) / res));
      const int gy = h - 1 - static_cast<int>(std::floor((msg->pose.position.y - oy) / res));
      //floor来自cmath,向下取整，四舍五入可能导致右偏一格
      if (gx < 0 || gx >= w || gy < 0 || gy >= h) 
      {
         return;
      }//判断有没有越界

      e_x = gx;
      e_y = gy;
      runAStar(s_x, s_y, e_x, e_y);//把目标传给寻路
   }
   //A*本质就是长了眼睛的D,我搓出过完整的D,A*是在它的基础上改的，同时把原本传入点的序号改成了坐标
   void runAStar(int s_x, int s_y, int e_x, int e_y)
   {
      g_cost.assign(h, std::vector<double>(w, 1e9));//这是洞潜的地图，在下水之前我默认洞里全是石头，下水后发现可以走的路就在地图上如实标记出来
      parent.assign(h, std::vector<std::pair<int,int>>(w,{-1,-1}));//这是洞潜时的安全绳
      path.clear();//queue不会自己清空
      move = std::priority_queue<::Node>();//为什么Node前面有:: ？是因为和ros的rclcpp::Node重名了，前面加上这个代表从全局里面找
                                           //其实可以直接把这个队列放到局部里面可以直接省掉清空流程，这里保留仅作为提醒
      g_cost[s_y][s_x]=0;
      move.push({s_y,s_x,heuristic(s_x,s_y,e_x,e_y),0.0});//起点的权重，是用来启动循环的
      while(move.empty()==0)
      {
         ::Node temp=move.top();
         move.pop();
         if(temp.g>g_cost[temp.y][temp.x])
         {
            continue;
         }//看看有没有过时的值,因为下方的算法可能会产生新的最小值，把旧的放在下面，导致产生过时的最小值
         for(int i=0;i<8;i++)
         {
            if(temp.y+dy[i]<0||temp.y+dy[i]>=h||temp.x+dx[i]<0||temp.x+dx[i]>=w)
            {
               continue;
            }//防止越界
            if(cmap[temp.y+dy[i]][temp.x+dx[i]]==255)
            {
               continue;
            }//防止撞墙

            if(g_cost[temp.y][temp.x]+cost[i]<g_cost[temp.y+dy[i]][temp.x+dx[i]])
            {
               g_cost[temp.y+dy[i]][temp.x+dx[i]]=g_cost[temp.y][temp.x]+cost[i];
               move.push({temp.y+dy[i],temp.x+dx[i],
                  g_cost[temp.y+dy[i]][temp.x+dx[i]]+heuristic(temp.x+dx[i],temp.y+dy[i],e_x,e_y),
                  g_cost[temp.y+dy[i]][temp.x+dx[i]]});
                  parent[temp.y+dy[i]][temp.x+dx[i]].first=temp.y;
                  parent[temp.y+dy[i]][temp.x+dx[i]].second=temp.x;//用安全绳记录走过的路径
            }//对每一个坐标的八个方向进行判断，从而找到最佳的那个方向
         }
         if(temp.y==e_y&&temp.x==e_x)
         {
            break;
         }//到终点就停下来
      }
       if (g_cost[e_y][e_x] >= 1e9)
      {
         RCLCPP_WARN(this->get_logger(), "目标不可达");
         path.clear();
         publishPath();// 发一个空Path，把RViz里的旧红线擦掉
         return;
      }
      int ty = e_y, tx = e_x;
      //下面的循环相当于沿着安全绳往回走
      while (true)
      {
          path.push_back({ty, tx});
          if (ty == s_y && tx == s_x) 
          {
            break;
          }//爬回起点了，跳出循环
          const std::pair<int, int> pre = parent[ty][tx];
          ty = pre.first;
          tx = pre.second;
      }
      std::reverse(path.begin(), path.end());//因为顺着绳子爬是倒着来的，但是机器人要正着走，所以翻一下顺序
      publishPath();
   }

   void publishPath()
   {
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = "map";//在map坐标系里进行
      path_msg.header.stamp = this->now();//拿系统时间

      nav2_costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
      const double res = cm->getResolution();
      const double ox  = cm->getOriginX();
      const double oy  = cm->getOriginY();

      for (auto& p : path)//范围for循环
      {
         geometry_msgs::msg::PoseStamped pose;
         pose.header.frame_id = "map";
         pose.pose.position.x = p.second * res + ox;
         pose.pose.position.y = (h-1-p.first) * res + oy;//之前的逆运算
         pose.pose.orientation.w = 1.0;//单位四元数(0,0,0,1)表示无旋转，不写默认全0，不是合法四元数
         path_msg.poses.push_back(pose);
      }
      path_pub_->publish(path_msg);
      RCLCPP_INFO(this->get_logger(), "OK %zu", path_msg.poses.size());
   }//关于为什么加header：
   //   相当于签合同的中间商，告诉rivz这个path数据来自哪里，什么时候来的

   std::vector<std::pair<int,int>> path;
   std::vector<std::vector<int>> cmap;
   std::vector<std::vector<double>> g_cost;
   std::priority_queue<::Node> move;
   std::vector<std::vector<std::pair<int,int>>> parent;
   const int    dx[8]   = {-1, 1, 0, 0, -1, -1, 1,  1};
   const int    dy[8]   = { 0, 0,-1, 1, -1,  1,-1,  1};
   const double cost[8] = { 1, 1, 1, 1, 1.4142135623730951, 1.4142135623730951, 1.4142135623730951, 1.4142135623730951};
   int h = 0;
   int w = 0;
   int s_x = 462;
   int s_y = 551;
   int e_x = -1;
   int e_y = -1;

   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
   nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
   rclcpp::TimerBase::SharedPtr timer_;
   bool ready_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavNode>();//我要用NavNode这个节点
    rclcpp::executors::MultiThreadedExecutor exec;//多线程运行，效率更高
    exec.add_node(node);
    exec.add_node(node->costmapNode()->get_node_base_interface());//把costmapNode的接口接上，上面那个不需要的原因是NavNode继承了来自父节点的接口
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
