#!/usr/bin/env python2
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
优化说明：X型全向轮专属 + 实时激光闭环防撞 + 方案1(虚拟障碍惩罚防原路返回) + 方案2(大位移纯侧滑脱困)
'''
import rospy
import actionlib
import tf
import math
import random
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
from TTS_audio.srv import StringService, StringServiceRequest

# ==================== 全局变量定义 ====================
clue = 1
task_numbers = []
points = [1, 2, 3, 4]
task_mapping = {
    31: 5,  32: 6,  33: 7,
    40: 8,  41: 9,  42: 10,
    49: 11, 50: 12, 51: 13
}

class NavigationDemoOptimized:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # 激光雷达订阅
        self.latest_scan = None
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        self.tf_listener = tf.TransformListener()
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base 服务...")
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("等待视觉大模型检测服务 /fruit_detection ...")
        rospy.wait_for_service('/fruit_detection', timeout=20)
        self.fruit_detection_service = rospy.ServiceProxy('/fruit_detection', Trigger)
        
        rospy.loginfo("等待 TTS 服务 tts_service ...")
        rospy.wait_for_service('tts_service', timeout=20)
        self.tts_service = rospy.ServiceProxy('tts_service', StringService)
        
        # 方案1核心：记录历史卡死点，用于对原路径施加临时惩罚
        self.stuck_history = []
        
        rospy.loginfo("系统初始化完毕，准备接收唤醒信号。")

    # -------------------- 激光雷达回调与扇区计算 --------------------
    def scan_cb(self, msg):
        self.latest_scan = msg

    def get_sector_min_range(self, deg_min, deg_max):
        """获取指定角度扇区内的最近障碍物距离（含角度归一化与死区过滤）"""
        if self.latest_scan is None:
            return 9.9  
        msg = self.latest_scan
        if msg.angle_increment == 0.0:
            return 9.9
            
        a0 = math.radians(deg_min)
        a1 = math.radians(deg_max)
        if a0 > a1:
            a0, a1 = a1, a0
            
        rmin = float("inf")
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r <= msg.range_min:
                continue
                
            a = msg.angle_min + i * msg.angle_increment
            while a > math.pi: a -= 2.0 * math.pi
            while a < -math.pi: a += 2.0 * math.pi
            
            if a0 <= a <= a1:
                if r < rmin:
                    rmin = r
        return rmin

    # -------------------- 语音播报 & 视觉调用 --------------------
    def tts_client(self, text):
        if isinstance(text, unicode):
            text = text.encode('utf-8')
        try:
            req = StringServiceRequest()
            req.data = text    
            self.tts_service(req)
            rospy.loginfo("TTS 播报成功: %s" % text)
            return True
        except Exception as e:
            rospy.logerr("TTS 失败: %s" % str(e))
            return False

    def call_fruit_detection_service(self):
        try:
            rospy.set_param('/detect', 1)
            rospy.sleep(0.8) 
            response = self.fruit_detection_service()
            rospy.loginfo("VLM 识别结果: %s" % response.message)
            return response.message.strip()
        except Exception as e:
            rospy.logerr("VLM 服务调用失败: %s" % e)
            return "无"

    # -------------------- 核心：智能导航与实时防撞监控 --------------------
    def get_robot_pose(self):
        try:
            self.tf_listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(0.5))
            trans, rot = self.tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            _, _, yaw = euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except Exception as e:
            rospy.logwarn_throttle(2.0, "TF 变换获取失败: %s", str(e))
            return None, None, None

    def goto(self, p):
        max_retries = 3 
        target_x, target_y, target_yaw_deg = p
        target_yaw_rad = math.radians(target_yaw_deg)

        for attempt in range(max_retries):
            rospy.loginfo("[Navi] 规划前往目标点: %s (尝试 %d/%d)" % (p, attempt + 1, max_retries))
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = target_x
            goal.target_pose.pose.position.y = target_y
            q = quaternion_from_euler(0.0, 0.0, target_yaw_rad)
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            
            self.move_base.send_goal(goal)
            start_time = rospy.Time.now().to_sec()
            last_move_time = start_time
            last_pose = self.get_robot_pose()
            
            # 监控 move_base 执行状态 + 实时激光防撞钩子
            while not rospy.is_shutdown():
                # 实时激光硬急刹保护
                front_danger = self.get_sector_min_range(-25, 25)
		left_danger  = self.get_sector_min_range(60, 120)
		right_danger = self.get_sector_min_range(-120, -60)
		rear_danger  = min(self.get_sector_min_range(150, 180), self.get_sector_min_range(-180, -150))
                if front_danger < 0.20 or left_danger < 0.20 or right_danger < 0.20 or rear_danger < 0.20:
                    rospy.logerr("[Safety] 🚨 导航途中前方 20cm 内检测到紧急障碍！立即急刹并脱困！")
                    self.move_base.cancel_goal()
                    self.pub.publish(Twist()) 
                    self.smart_escape()       
                    break

                state = self.move_base.get_state()
                if state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED]:
                    break
                    
                current_time = rospy.Time.now().to_sec()
                rx, ry, ryaw = self.get_robot_pose()
                
                if rx is not None and last_pose[0] is not None:
                    dist_moved = math.hypot(rx - last_pose[0], ry - last_pose[1])
                    if dist_moved > 0.05:
                        last_pose = (rx, ry, ryaw)
                        last_move_time = current_time
                    elif (current_time - last_move_time) > 7.0: # 稍微调敏感一点，7秒不动即判定死锁
                        rospy.logwarn("[Navi] 判定陷入局部死锁！取消目标。")
                        self.move_base.cancel_goal()
                        break
                        
                if (current_time - start_time) > 45.0:
                    rospy.logwarn("[Navi] 全局超时。")
                    self.move_base.cancel_goal()
                    break
                rospy.sleep(0.1)
                
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("move_base 成功到达目标点 %s!" % p)
                return True
                
            # 最后一公里强行接管
            rx, ry, _ = self.get_robot_pose()
            if rx is not None:
                dist_to_goal = math.hypot(rx - target_x, ry - target_y)
                if dist_to_goal < 0.65:
                    rospy.logwarn("[Navi] 距离目标仅 %.2fm，触发末端强制全向靠泊...", dist_to_goal)
                    if self.local_smart_docking(target_x, target_y, target_yaw_rad):
                        return True
            
            # 触发全向智能脱困与方案1惩罚
            if attempt < max_retries - 1:
                if rx is not None:
                    self.stuck_history.append((rx, ry)) # 记录卡死坐标
                self.smart_escape()
                
        rospy.logerr("[Navi] 彻底失败，放弃目标点 %s" % p)
        return False

    def local_smart_docking(self, tx, ty, tyaw):
        """X型麦克纳姆轮专属全向靠泊解算"""
        rate = rospy.Rate(15)
        start_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            rx, ry, ryaw = self.get_robot_pose()
            if rx is None:
                continue
                
            ex_map = tx - rx
            ey_map = ty - ry
            yaw_err = math.atan2(math.sin(tyaw - ryaw), math.cos(tyaw - ryaw))
            dist = math.hypot(ex_map, ey_map)
            
            if dist < 0.05 and abs(yaw_err) < 0.08:
                rospy.loginfo("[Docking] 全向靠泊精准到位！")
                self.pub.publish(Twist())
                return True
                
            if rospy.Time.now().to_sec() - start_time > 10.0:
                rospy.logwarn("[Docking] 靠泊超时。")
                self.pub.publish(Twist())
                return False

            cos_y = math.cos(ryaw)
            sin_y = math.sin(ryaw)
            v_front = cos_y * ex_map + sin_y * ey_map
            v_left  = -sin_y * ex_map + cos_y * ey_map
            
            cmd = Twist()
            cmd.linear.x = max(-0.25, min(0.25, v_front * 1.5))
            cmd.linear.y = max(-0.25, min(0.25, v_left * 1.5))
            cmd.angular.z = max(-0.45, min(0.45, yaw_err * 1.8))
            
            front = self.get_sector_min_range(-20, 20)
            rear = min(self.get_sector_min_range(160, 180), self.get_sector_min_range(-180, -160))
            left = self.get_sector_min_range(70, 110)
            right = self.get_sector_min_range(-110, -70)
            
            if cmd.linear.x > 0 and front < 0.18: cmd.linear.x = 0.0
            if cmd.linear.x < 0 and rear < 0.18: cmd.linear.x = 0.0
            if cmd.linear.y > 0 and left < 0.18: cmd.linear.y = 0.0
            if cmd.linear.y < 0 and right < 0.18: cmd.linear.y = 0.0
                
            self.pub.publish(cmd)
            rate.sleep()
        return False

    def smart_escape(self):
        """
        X型麦轮专属精准脱困：
        1. 空间裕度最大化：智能比对前后左右的真实距离，永远往最开阔、障碍物最少的地方逃逸；
        2. 安全距离限制：缩短脱困执行时间，防止侧移距离过大而撞墙。
        """
        rospy.logwarn("[Escape] 启动 X 型全向轮智能寻优脱困策略...")
        
        # 1. 扫描四个方向的最近障碍距离
        front = self.get_sector_min_range(-30, 30)
        rear = min(self.get_sector_min_range(150, 180), self.get_sector_min_range(-180, -150))
        left = self.get_sector_min_range(60, 120)
        right = self.get_sector_min_range(-120, -60)
        
        front = 9.9 if math.isinf(front) else front
        rear = 9.9 if math.isinf(rear) else rear
        left = 9.9 if math.isinf(left) else left
        right = 9.9 if math.isinf(right) else right
        
        rospy.loginfo("雷达实时空间感知 -> 前:%.2f 后:%.2f 左:%.2f 右:%.2f" % (front, rear, left, right))
        
        cmd = Twist()
        direction = ""
        
        # 2. 空间裕度最优选择：找出四个方向中空间最大的那个方向进行逃逸
        # 设置基础安全门槛（必须大于 0.35米 才考虑作为逃逸通道）
        safe_threshold = 0.35
        
        # 找出最大可用空间的维度
        max_space = max(front, rear, left, right)
        
        if max_space < safe_threshold:
            rospy.logwarn("[Escape] ⚠️ 四周空间均极度狭窄 (<35cm)，执行最小位移倒退脱困")
            cmd.linear.x = -0.15
            direction = "rear"
        elif max_space == left and left >= safe_threshold:
            rospy.loginfo("[Escape] 🎯 左侧空间最大 (%.2fm)，向左精准侧滑脱困", left)
            cmd.linear.y = 0.2  # 降低速度，缩短位移
            direction = "left"
        elif max_space == right and right >= safe_threshold:
            rospy.loginfo("[Escape] 🎯 右侧空间最大 (%.2fm)，向右精准侧滑脱困", right)
            cmd.linear.y = -0.2 
            direction = "right"
        elif max_space == rear and rear >= safe_threshold:
            rospy.loginfo("[Escape] 🎯 后方空间最大 (%.2fm)，向后直线退避脱困", rear)
            cmd.linear.x = -0.2
            direction = "rear"
        else:
            rospy.loginfo("[Escape] 🎯 前方空间最大 (%.2fm)，向前缓慢前移脱困", front)
            cmd.linear.x = 0.2
            direction = "front"
            
        # 3. 严格限制执行时间（从 2.2 秒缩短至 1.2 秒，防止侧移距离过大撞墙）
        rate = rospy.Rate(10)
        for _ in range(12):  # 12次 * 0.1秒 = 1.2秒，侧移距离控制在约 20~25cm
            # 闭环防撞：在逃逸过程中，如果朝向的目标方向突然出现 < 0.18m 的障碍物，瞬间急刹
            if direction == "rear" and min(self.get_sector_min_range(150, 180), self.get_sector_min_range(-180, -150)) < 0.18: break
            if direction == "left" and self.get_sector_min_range(60, 120) < 0.18: break
            if direction == "right" and self.get_sector_min_range(-120, -60) < 0.18: break
            if direction == "front" and self.get_sector_min_range(-30, 30) < 0.18: break
                
            self.pub.publish(cmd)
            rate.sleep()
            
        self.pub.publish(Twist()) # 强制刹停
        
        # 4. 清理代价地图并加入微小扰动以打破原路径规划重合
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmaps()
            rospy.loginfo("[Escape] 已彻底清空代价地图")
        except:
            pass
            
        # 微调朝向，迫使全局规划器重新生成不同的走法
        rx, ry, _ = self.get_robot_pose()
        if rx is not None:
            turn_cmd = Twist()
            turn_cmd.angular.z = 0.3 if random.random() > 0.5 else -0.3
            for _ in range(4):
                self.pub.publish(turn_cmd)
                rate.sleep()
            self.pub.publish(Twist())
            
        rospy.sleep(0.5)

    # -------------------- 任务逻辑控制 --------------------
    def mission(self, point):
        global clue
        rospy.loginfo(">>> 前往外围检测点: %s" % point)
        if self.goto(goals[point]):
            detect_result = self.call_fruit_detection_service()
            if detect_result.isdigit():
                task_id = int(detect_result)
                if task_id in task_mapping:
                    task_numbers.append(task_id)
                    self.tts_client(u"已检测第%d条线索为%d号" % (clue, task_id))
                    clue += 1 
                else:
                    rospy.logwarn("识别到未定义任务编号: %d", task_id)
            else:
                self.tts_client(u"当前点未能有效识别")
        else:
            rospy.logwarn("未能到达外围检测点 %s", point)

    def start_mission_callback(self, msg):
        if msg.data == "start":
            rospy.loginfo("=== 接收到唤醒信号，任务启动 ===")
            
            start_cmd = Twist()
            start_cmd.linear.x = 0.2  
            for _ in range(25):       
                self.pub.publish(start_cmd)
                rospy.sleep(0.1)
            self.pub.publish(Twist()) 
            rospy.sleep(0.5)
            
            for i, p in enumerate(points):
                self.mission(p)
                
            rospy.loginfo("收集到的任务点: %s", task_numbers)
            
            for task_id in task_numbers:
                goal_index = task_mapping.get(task_id)
                if goal_index is not None:
                    rospy.loginfo("-> 导航前往任务点: %d号", task_id)
                    if self.goto(goals[goal_index]):
                        rospy.sleep(1.0)
                        self.tts_client(u"已到达任务点%d号" % task_id)
            
            rospy.loginfo("=== 准备冲刺终点 ===")
            self.tts_client(u"已完成所有任务点，正在返航")
            self.goto([0.01, 3.32, -90.0])
            self.tts_client(u"已到达终点，比赛结束")

if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)
    try:
        x_list = [float(x.strip()) for x in rospy.get_param('~goalListX').split(",") if x.strip()]
        y_list = [float(y.strip()) for y in rospy.get_param('~goalListY').split(",") if y.strip()]
        yaw_list = [float(yaw.strip()) for yaw in rospy.get_param('~goalListNavYaw' if rospy.has_param('~goalListNavYaw') else '~goalListYaw').split(",") if yaw.strip()]
        goals = [[x, y, yaw] for x, y, yaw in zip(x_list, y_list, yaw_list)]
    except Exception as e:
        rospy.logerr("解析点位参数失败: %s" % e)
        exit(1)

    navi = NavigationDemoOptimized()
    rospy.Subscriber('/start_mission', String, navi.start_mission_callback)
    rospy.spin()
