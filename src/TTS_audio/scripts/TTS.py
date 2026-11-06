#!/usr/bin/env python3
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
company:WCXC
'''
import rospy
import asyncio  # 异步IO库
import websockets  # WebSocket库
import uuid  # 生成唯一ID
import json  # JSON处理
import gzip  # 数据压缩
import copy  # 深拷贝
import os
from TTS_audio.srv import StringService, StringServiceResponse

# 消息类型映射
MESSAGE_TYPES = {
    11: "音频服务器响应",
    12: "前端服务器响应",
    15: "服务器错误消息"
}

# 消息类型特定标志映射
MESSAGE_TYPE_SPECIFIC_FLAGS = {
    0: "无序列号",
    1: "序列号 > 0",
    2: "服务器最后一条消息 (序列号 < 0)",
    3: "序列号 < 0"
}

# 消息序列化方法映射
MESSAGE_SERIALIZATION_METHODS = {
    0: "无序列化",
    1: "JSON",
    15: "自定义类型"
}

# 消息压缩方法映射
MESSAGE_COMPRESSIONS = {
    0: "无压缩",
    1: "gzip",
    15: "自定义压缩方法"
}

# 应用配置
appid = "1344852331"  # 应用ID
token = "lpmnfDfA0tE2nSPXpgliHkRDbWL0r_SF"  # 访问令牌
cluster = "volcano_tts"  # 集群名称
voice_type = "zh_female_vv_uranus_bigtts"  # 语音类型
host = "openspeech.bytedance.com"  # 服务器主机
api_url = f"wss://{host}/api/v1/tts/ws_binary"  # WebSocket API URL

# 默认请求头
default_header = bytearray(b'\x11\x10\x11\x00')

# 请求JSON模板
request_json = {
    "app": {
        "appid": appid,
        "token": token,  # 直接使用配置的token
        "cluster": cluster
    },
    "user": {
        "uid": "2132711847"  # 用户ID
    },
    "audio": {
        "voice_type": voice_type,  # 语音类型
        "encoding": "mp3",  # 音频编码格式
        "speed_ratio": 0.9,  # 语速比例
        "volume_ratio": 2.0,  # 音量比例
        "pitch_ratio": 1.0,  # 音高比例
    },
    "request": {
        "reqid": "uuid",  # 请求ID占位符
        "text": "字节跳动语音合成。",  # 待合成文本
        "text_type": "plain",  # 文本类型
        "operation": "submit"  # 操作类型
    }
}

async def send_tts_request(text):
    """
    发送TTS请求到服务器并保存音频文件
    :param text: 待合成的文本
    :return: 保存的音频文件路径
    """
    # 复制请求模板并填充数据
    submit_request_json = copy.deepcopy(request_json)
    submit_request_json["request"]["reqid"] = str(uuid.uuid4())  # 生成唯一请求ID
    submit_request_json["request"]["text"] = text  # 设置待合成文本

    # 将JSON序列化为字节并压缩
    payload_bytes = str.encode(json.dumps(submit_request_json))
    payload_bytes = gzip.compress(payload_bytes)

    # 构建完整请求
    full_client_request = bytearray(default_header)
    full_client_request.extend((len(payload_bytes)).to_bytes(4, 'big'))  # 添加负载大小
    full_client_request.extend(payload_bytes)  # 添加负载数据

    # 设置请求头
    header = {"Authorization": f"Bearer; {token}"}

    # 连接到WebSocket服务器并发送请求
    async with websockets.connect(api_url, extra_headers=header, ping_interval=None) as ws:
        await ws.send(full_client_request)  # 发送请求
        file_to_save = open("output.mp3", "wb")  # 打开文件以保存音频
        while True:
            res = await ws.recv()  # 接收服务器响应
            done = parse_response(res, file_to_save)  # 解析响应
            if done:
                file_to_save.close()  # 关闭文件
                break
        return "output.mp3"  # 返回保存的音频文件路径

def parse_response(res, file):
    """
    解析服务器响应
    :param res: 服务器返回的原始字节数据
    :param file: 用于保存音频数据的文件对象
    :return: 是否完成解析（True/False）
    """
    # 解析协议头
    protocol_version = res[0] >> 4  # 协议版本
    header_size = res[0] & 0x0f  # 头部大小
    message_type = res[1] >> 4  # 消息类型
    message_type_specific_flags = res[1] & 0x0f  # 消息类型特定标志
    serialization_method = res[2] >> 4  # 序列化方法
    message_compression = res[2] & 0x0f  # 压缩方法
    reserved = res[3]  # 保留字段
    header_extensions = res[4:header_size * 4]  # 头部扩展
    payload = res[header_size * 4:]  # 负载数据

    # 处理音频服务器响应
    if message_type == 0xb:  # 音频服务器响应
        if message_type_specific_flags == 0:  # 无序列号（ACK）
            return False
        else:
            sequence_number = int.from_bytes(payload[:4], "big", signed=True)  # 序列号
            payload_size = int.from_bytes(payload[4:8], "big", signed=False)  # 负载大小
            payload = payload[8:]  # 实际负载数据
        file.write(payload)  # 写入音频数据
        if sequence_number < 0:  # 如果是最后一条消息
            return True
        else:
            return False
    elif message_type == 0xf:  # 错误消息
        return True
    elif message_type == 0xc:  # 前端服务器响应
        return False
    else:  # 未定义的消息类型
        return True

def handle_tts_request(req):
    """处理TTS服务请求，调用异步的WebSocket TTS接口"""
    TEXT = req.data
    rospy.loginfo(f"收到TTS请求: {TEXT}")
    
    try:
        # 运行异步函数
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        audio_path = loop.run_until_complete(send_tts_request(TEXT))
        loop.close()
        
        # 播放生成的音频
        rospy.loginfo(f"音频保存至: {audio_path}")
        os.system(f'mplayer {audio_path}')  # 确保系统安装了mplayer
        return StringServiceResponse("TTS处理完成")
    except Exception as e:
        rospy.logerr(f"TTS处理出错: {str(e)}")
        return StringServiceResponse(f"错误: {str(e)}")

def tts_server():
    rospy.init_node('tts_server')
    s = rospy.Service('tts_service', StringService, handle_tts_request)
    rospy.loginfo("TTS服务已启动，等待请求...")
    rospy.spin()

if __name__ == '__main__':
    tts_server()
