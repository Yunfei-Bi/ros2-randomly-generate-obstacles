import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
import random
import time

class GazeboObjectManager(Node):
    def __init__(self):
        super().__init__('gazebo_object_manager')
        self.get_logger().info('初始化Gazebo对象管理器')
        
        # 等待节点完全初始化
        self.get_logger().info('等待节点完全初始化...')
        time.sleep(1.0)
        
        # 创建服务客户端
        self.get_logger().info('创建/spawn_entity服务客户端...')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        self.get_logger().info('创建/delete_entity服务客户端...')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # 等待服务可用
        self.get_logger().info('等待/spawn_entity服务可用（超时时间300秒）...')
        if not self.spawn_client.wait_for_service(timeout_sec=300):
            self.get_logger().error('Spawn服务不可用，退出')
            
            # 列出所有可用的服务，用于调试
            self.get_logger().info('可用的服务列表:')
            services = self.get_service_names_and_types()
            for service_name, service_types in services:
                self.get_logger().info(f'  - {service_name}: {service_types}')
            
            # 检查/spawn_entity服务是否在列表中
            service_found = False
            for service_name, _ in services:
                if service_name == '/spawn_entity':
                    service_found = True
                    break
            
            if service_found:
                self.get_logger().error('服务列表中存在/spawn_entity，但无法连接，可能是类型不匹配')
            else:
                self.get_logger().error('服务列表中未找到/spawn_entity')
                
            exit(1)
        
        self.get_logger().info('等待/delete_entity服务可用（超时时间300秒）...')
        if not self.delete_client.wait_for_service(timeout_sec=300):
            self.get_logger().error('Delete服务不可用，退出')
            exit(1)
            
        self.get_logger().info('所有服务已成功连接')
        
    def delete_object(self, name):
        """同步删除障碍物（阻塞直到响应）"""
        req = DeleteEntity.Request()
        req.name = name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)  # 阻塞等待响应
        
        if future.result() is not None:
            self.get_logger().info(f"删除 {name} 结果: {future.result().success}")
            return future.result().success
        else:
            self.get_logger().error(f"删除 {name} 失败: {future.exception()}")
            return False

    def spawn_cylinder(self, name, x, y, z=2.0, radius=1.0, height=0.5):
        """同步生成圆柱体，返回服务响应（半径支持随机生成）"""
        self.get_logger().info(f'准备生成圆柱体: {name} 在位置 ({x}, {y}, {z})，半径: {radius:.2f}')
        
        sdf_model = f"""
        <sdf version="1.6">
          <model name="{name}">
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>{radius}</radius>
                    <length>{height}</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>{radius}</radius>
                    <length>{height}</length>
                  </cylinder>
                </geometry>
                <material>
                  <script>
                    <name>Gazebo/Red</name>
                  </script>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf_model
        req.initial_pose = pose
        
        # 同步调用服务
        self.get_logger().info(f'发送生成请求: {name}')
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)  # 阻塞等待响应
        
        if future.result() is not None:
            self.get_logger().info(f"生成 {name} 结果: {future.result().success}")
            return future.result().success
        else:
            self.get_logger().error(f"生成 {name} 失败: {future.exception()}")
            return False

    def delete_object(self, name):
        """同步删除障碍物，返回服务响应"""
        self.get_logger().info(f'准备删除对象: {name}')
        
        req = DeleteEntity.Request()
        req.name = name
        
        # 同步调用服务
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'删除结果: {future.result().success}, 消息: {future.result().status_message}')
            return future.result()
        else:
            self.get_logger().error(f'服务调用失败: {future.exception()}')
            return None

    def generate_random_obstacles(self, num_obstacles, x_range=(0, 40), y_range=(0, 40)):
        """生成随机障碍物（同步版本），半径在1~2之间随机，默认范围x轴(0,40)，y轴(0,40)"""
        self.get_logger().info(f"开始生成 {num_obstacles} 个障碍物，范围: x={x_range}, y={y_range}")
        obstacles = []
        
        # 同步删除旧障碍物
        for i in range(num_obstacles):
            name = f"cylinder_{i}"
            self.get_logger().info(f"删除旧障碍物: {name}")
            if self.delete_object(name):
                self.get_logger().info(f"成功删除 {name}")
            else:
                self.get_logger().info(f"删除 {name} 失败，可能不存在")
        
        # 同步生成新障碍物
        for i in range(num_obstacles):
            x = random.uniform(*x_range)
            y = random.uniform(*y_range)
            # 生成1~2之间的随机半径
            radius = random.uniform(1.0, 2.0)
            name = f"cylinder_{i}"
            self.get_logger().info(f"生成新障碍物: {name} 在 ({x:.2f}, {y:.2f})，半径: {radius:.2f}")
            if self.spawn_cylinder(name, x, y, radius=radius):
                obstacles.append((name, x, y, radius))  # 保存半径信息
                self.get_logger().info(f"成功生成 {name}")
            else:
                self.get_logger().error(f"生成 {name} 失败")
        
        return obstacles


# 训练循环保持同步逻辑
def training_loop():
    rclpy.init()
    manager = GazeboObjectManager()
    try:
        for episode in range(20):
            manager.get_logger().info(f"第 {episode+1} 轮开始")
            # 使用新的范围参数生成障碍物
            obstacles = manager.generate_random_obstacles(5, x_range=(0, 40), y_range=(0, 40))
            print(f"生成 {len(obstacles)} 个障碍物")
            for name, x, y, radius in obstacles:
                print(f"  - {name}: 位置({x:.2f}, {y:.2f})，半径{radius:.2f}")
            
            # 模拟训练步骤（同步执行）
            for step in range(20):
                rclpy.spin_once(manager, timeout_sec=0.1)
                print(f"轮 {episode+1}, 步 {step+1}")
            print("\n")
    except KeyboardInterrupt:
        print("训练中断")
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    training_loop()