import socket
import time

# --- 配置区 ---
REFEREE_IP = '192.168.5.2'  # 你的电脑裁判系统 IP
REFEREE_PORT = 8888         # 裁判系统监听的端口

def start_client():
    while True:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"[*] 尝试连接到裁判系统 {REFEREE_IP}:{REFEREE_PORT}...")
        
        try:
            # 尝试连接
            client_socket.connect((REFEREE_IP, REFEREE_PORT))
            print("[+] 连接成功！小车已上线。")
            
            while True:
                # 1. 模拟发送心跳包或测试数据
                # 注意：实际比赛需要按照官方协议发送十六进制 (Hex) 数据
                test_msg = "Hello Referee, I am Car!\n"
                client_socket.sendall(test_msg.encode('utf-8'))
                print(f"[->] 发送数据: {test_msg.strip()}")
                
                # 2. 接收裁判系统的指令 (非阻塞式接收可后续再加，这里先用阻塞式测试)
                client_socket.settimeout(2.0) # 设置2秒超时，防止一直卡在这里
                try:
                    response = client_socket.recv(1024)
                    if response:
                        print(f"[<-] 收到指令: {response.decode('utf-8')}")
                    else:
                        print("[-] 裁判系统主动断开了连接。")
                        break # 跳出内层循环，触发重新连接
                except socket.timeout:
                    # 没收到数据正常，继续下一轮循环
                    pass
                
                time.sleep(2) # 每两秒发送一次，防止刷屏太快
                
        except ConnectionRefusedError:
            print("[-] 连接被拒绝，裁判系统没开，或者IP/端口不对。")
        except Exception as e:
            print(f"[-] 发生错误: {e}")
        finally:
            client_socket.close()
            print("[*] 5秒后尝试重新连接...\n")
            time.sleep(5)

if __name__ == '__main__':
    start_client()
