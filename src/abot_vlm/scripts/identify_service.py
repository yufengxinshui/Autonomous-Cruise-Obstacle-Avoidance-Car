#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
import rospy
import cv2
import numpy as np
from PIL import Image, ImageFont, ImageDraw
import time
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerResponse
from API_KEY import *
import json
from openai import OpenAI
import base64
import sys
import re

result_pub = None

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    # 如果消息和系统的字节序不同
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # 转换编码格式
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = image_opencv, cv2.COLOR_GRAY2BGR
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None

    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = ROSImage()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

def top_view_shot(image_msg):
    global detect
    # 将ROS图像消息转换为OpenCV格式
    img_bgr = imgmsg_to_cv2(image_msg)
    # 从参数服务器获取detect的值
    detect = rospy.get_param('/detect', 255)
    
    if detect == 1:
        # 保存图像
        save_path = '/home/abot/xiangcai/664486/src/abot_vlm/temp/vl_now.jpg'
        rospy.loginfo(f'保存至{save_path}')
        cv2.imwrite(save_path, img_bgr)
        # 重置detect
        rospy.set_param('/detect', 255)
        cv2.waitKey(1)

        # 调用视觉大模型API
        result = yi_vision_api()
        result_pub.publish(result)
        rospy.loginfo(f'最终识别结果：{result}')

def yi_vision_api(
    PROMPT='图中有一道数学题。先写出解题过程，最后一行只输出答案：如果答案存在，只输出纯阿拉伯数字；如果没有符合条件的答案，不要输出任何文字、汉字、符号，不允许输出“无”。',
    img_path='/home/abot/xiangcai/664486/src/abot_vlm/temp/vl_now.jpg',
    max_retry=3  # 最大重试次数，避免无限循环
):
    '''
    视觉大模型API，自动提取结果里的纯数字
    '''
    # 初始化OpenAI客户端
    client = OpenAI(
        api_key=YI_KEY,
        base_url="https://ark.cn-beijing.volces.com/api/v1"
    )
    
    # 编码图片为base64
    try:
        with open(img_path, 'rb') as image_file:
            image_base64 = base64.b64encode(image_file.read()).decode('utf-8')
        image_url = f'data:image/jpeg;base64,{image_base64}'
    except Exception as e:
        rospy.logerr(f'读取图片失败：{e}')
        return ""   # 找不到图返回空串
    
    retry_count = 0
    while retry_count < max_retry:
        try:
            response = client.chat.completions.create(
                #在这里更换
                model="ep-20261007164945-g52wz",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": PROMPT},
                            {"type": "image_url", "image_url": {"url": image_url}}
                        ]
                    }
                ],
                timeout=30
            )
            
            result_str = response.choices[0].message.content.strip()
            rospy.loginfo(f'大模型原始返回：\n{result_str}')

            # 正则提取所有连续阿拉伯数字，取最后一组作为答案
            all_nums = re.findall(r"\d+", result_str)
            if all_nums:
                clean_result = all_nums[-1]
            else:
                clean_result = ""

            rospy.loginfo(f'--- 清理后纯数字结果：[{clean_result}]')
            return clean_result

        except Exception as e:
            retry_count += 1
            rospy.logerr(f'调用大模型失败：{e}，第{retry_count}次重试...')
            time.sleep(1)
    
    # 重试耗尽返回空字符串
    rospy.logerr(f'超过最大重试次数（{max_retry}次），返回空')
    return ""

def handle_fruit_detection(req):
    # 调用视觉大模型API
    result = yi_vision_api()
    result_pub.publish(result)
    return TriggerResponse(success=True, message=result)

def main():
    global detect,result_pub
    rospy.init_node('identify_node', anonymous=True)
    # 订阅图像话题
    rospy.Subscriber('/usb_cam/image_raw', ROSImage, top_view_shot)
    # 初始化参数服务器
    rospy.set_param('/detect', 255)
    # 创建服务
    s = rospy.Service('fruit_detection', Trigger, handle_fruit_detection)
    result_pub = rospy.Publisher('/vision_recognition_result', String, queue_size=10)
    rospy.loginfo('视觉大模型模块启动成功！')
    rospy.loginfo('准备识别...')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
