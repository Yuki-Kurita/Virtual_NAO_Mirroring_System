# coding:utf-8
from controller import Robot, Motion, Keyboard
import sys
import os
import time
import math
import datetime
import socket
import numpy as np

class naoMirroring():
    def __init__(self):
        self.robot = Robot()
        # Motorの取得
        self.RSRMotor = self.robot.getMotor("RShoulderRoll")
        self.RSPMotor = self.robot.getMotor("RShoulderPitch")
        self.RERMotor = self.robot.getMotor("RElbowRoll")
        self.REYMotor = self.robot.getMotor("RElbowYaw")
        self.LSRMotor = self.robot.getMotor("LShoulderRoll")
        self.LSPMotor = self.robot.getMotor("LShoulderPitch")
        self.LERMotor = self.robot.getMotor("LElbowRoll")
        self.LEYMotor = self.robot.getMotor("LElbowYaw")
        
    def tcpip_config(self):
        k_dic = {}
        host = "163.221.38.217" #お使いのサーバーのホスト名を入れます
        port = 9876 #クライアントで設定したPORTと同じもの指定してあげます
        serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serversock.bind((host,port)) #IPとPORTを指定してバインドします
        serversock.listen(10) #接続の待ち受けをします（キューの最大数を指定）
        print ('Waiting for connections...')
        clientsock, client_address = serversock.accept() #接続されればデータを格納
        print("Success connection!")
        return clientsock
        

    def angleRShoulderPitch(self, x2, y2, z2, x1, y1, z1):
        # 肘が肩の位置よりも下の時
        if(y2<y1):
            angle = math.atan(abs(y2 - y1) / abs(z2 - z1))
            angle = math.degrees(angle)
            angle = -(angle)
            if(angle<-118):
                angle = -117
            return angle
        # 肘が肩の位置よりも上
        else:
            angle = math.atan((z2-z1)/(y2-y1))
            angle = math.degrees(angle)
            angle = 90-angle
            if angle < 90:
                return angle
            else:
                return 90
    
    # x2,y2,z2->shoulder, x1, y1, z1->elbow
    def angleRShoulderRoll(self, x2, y2, z2, x1, y1, z1, y, z):
        y_cross_z = np.cross(np.array(y), np.array(z))
        r_45 = [x1-x2, y1-y2, z1-z2]
        num = y_cross_z[0]*r_45[0] + y_cross_z[1]*r_45[1] + y_cross_z[2]*r_45[2]
        nolm_ycz = math.sqrt(pow(y_cross_z[0], 2) + pow(y_cross_z[1], 2) + pow(y_cross_z[2], 2))
        nolm_r45 = math.sqrt(pow(r_45[0], 2) + pow(r_45[1], 2) + pow(r_45[2], 2))
        den = nolm_ycz * nolm_r45
        radian = -(math.pi/2 - math.acos(num/den))
        # RShoulderは-をつける
        angle = -math.degrees(radian)
        # print("RShoulderRoll : {}".format(angle))
        return angle

    def angleLShoulderPitch(self, x2, y2, z2, x1, y1, z1):
        # 肘が肩の位置よりも下の時
        if (y2 < y1):
            angle = math.atan(abs(y2 - y1) / abs(z2 - z1))
            angle = math.degrees(angle)
            angle = -(angle)
            if (angle < -118):
                angle = -117
            return angle
        # 肘が肩の位置よりも上
        else:
            angle = math.atan((z2 - z1) / (y2 - y1))
            angle = math.degrees(angle)
            angle = 90 - angle
            # 肩関節が後ろに回らないための工夫
            if angle < 90:
                return angle
            else:
                return 90

    # x2,y2,z2->shoulder, x1, y1, z1->elbow
    def angleLShoulderRoll(self, x2, y2, z2, x1, y1, z1, y, z):
        y_cross_z = np.cross(np.array(y), np.array(z))
        r_45 = [x1-x2, y1-y2, z1-z2]
        num = y_cross_z[0]*r_45[0] + y_cross_z[1]*r_45[1] + y_cross_z[2]*r_45[2]
        nolm_ycz = math.sqrt(pow(y_cross_z[0], 2) + pow(y_cross_z[1], 2) + pow(y_cross_z[2], 2))
        nolm_r45 = math.sqrt(pow(r_45[0], 2) + pow(r_45[1], 2) + pow(r_45[2], 2))
        den = nolm_ycz * nolm_r45
        radian = -(math.pi/2 - math.acos(num/den))
        # LShoulderは-をつけない
        angle = -math.degrees(radian)
        # print("LShoulderRoll : {}".format(angle))
        return angle

    def angleRElbowYaw(self, x3, y3, z3, x2, y2, z2, x1, y1, z1, R, P, z): #calulates the ElbowYaw value for the Left elbow by using geometry
        b_rot_matrix_left  = np.array([[math.cos(R), -math.sin(R), 0],
                                      [math.sin(R),  math.cos(R), 0],
                                      [0, 0, 1]])
        b_rot_matrix_right = np.array([[ math.cos(P), 0, math.sin(P)],
                                       [0, 1, 0],
                                       [-math.sin(P), 0, math.cos(P)]])
        b_rot_matrix = np.dot(b_rot_matrix_left, b_rot_matrix_right)
        b = np.dot(b_rot_matrix, z)
        
        r_89  = [x2 - x3, y2 - y3, z2 - z3]
        r_910 = [x1 - x2, y1 - y2, z1 - z2]
        r_num = np.cross(np.array(r_89), np.array(r_910))
        r_den = math.sqrt(pow(r_num[0], 2) + pow(r_num[1], 2) + pow(r_num[2], 2))
        r = [r_num[0]/r_den, r_num[1]/r_den, r_num[2]/r_den]
        
        b_r_num = b[0]*r[0] + b[1]*r[1] + b[2]*r[2]
        nolm_b = math.sqrt(pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2))
        nolm_r = math.sqrt(pow(r[0], 2) + pow(r[1], 2) + pow(r[2], 2))
        b_r_den = nolm_b * nolm_r
        
        radian = (math.pi/2 - math.acos(b_r_num/b_r_den))
        angle = math.degrees(radian)
        # print("RElbowYaw : {}".format(angle))
        return angle

    # x3 -> shoulder : 4, x2 -> elbow : 5, x1 -> wrist : 6
    def angleRElbowRoll(self, x3, y3, z3, x2, y2, z2, x1, y1, z1): 
        r_45 = [x2 - x3, y2 - y3, z2 - z3]
        r_56 = [x1 - x2, y1 - y2, z1 - z2]
        num = r_45[0]*r_56[0] + r_45[1]*r_56[1] + r_45[2]*r_56[2]
        nolm_r_45 = math.sqrt(pow(r_45[0], 2) + pow(r_45[1], 2) + pow(r_45[2], 2))
        nolm_r_56 = math.sqrt(pow(r_56[0], 2) + pow(r_56[1], 2) + pow(r_56[2], 2))
        den = nolm_r_45 * nolm_r_56
        radian = math.acos(num / den)
        angle = math.degrees(radian)
        # print("RElbowRoll : {}".format(angle))
        return angle


    def angleLElbowYaw(self, x3, y3, z3, x2, y2, z2, x1, y1, z1, R, P, z): #calulates the ElbowYaw value for the Left elbow by using geometry
        b_rot_matrix_left  = np.array([[math.cos(R), -math.sin(R), 0],
                                      [math.sin(R),  math.cos(R), 0],
                                      [0, 0, 1]])
        b_rot_matrix_right = np.array([[ math.cos(P), 0, math.sin(P)],
                                       [0, 1, 0],
                                       [-math.sin(P), 0, math.cos(P)]])
        b_rot_matrix = np.dot(b_rot_matrix_left, b_rot_matrix_right)
        b = np.dot(b_rot_matrix, z)
        
        r_89  = [x2 - x3, y2 - y3, z2 - z3]
        r_910 = [x1 - x2, y1 - y2, z1 - z2]
        r_num = np.cross(np.array(r_89), np.array(r_910))
        r_den = math.sqrt(pow(r_num[0], 2) + pow(r_num[1], 2) + pow(r_num[2], 2))
        r = [r_num[0]/r_den, r_num[1]/r_den, r_num[2]/r_den]
        
        b_r_num = b[0]*r[0] + b[1]*r[1] + b[2]*r[2]
        nolm_b = math.sqrt(pow(b[0], 2) + pow(b[1], 2) + pow(b[2], 2))
        nolm_r = math.sqrt(pow(r[0], 2) + pow(r[1], 2) + pow(r[2], 2))
        b_r_den = nolm_b * nolm_r
        
        radian = (math.pi/2 - math.acos(b_r_num/b_r_den))
        angle = math.degrees(radian)
        print("LElbowYaw : {}".format(angle))
        return angle

    # x3 -> shoulder : 4, x2 -> elbow : 5, x1 -> wrist : 6
    def angleLElbowRoll(self, x3, y3, z3, x2, y2, z2, x1, y1, z1): 
        r_89 = [x2 - x3, y2 - y3, z2 - z3]
        r_910 = [x1 - x2, y1 - y2, z1 - z2]
        num = r_89[0]*r_910[0] + r_89[1]*r_910[1] + r_89[2]*r_910[2]
        nolm_r_89 = math.sqrt(pow(r_89[0], 2) + pow(r_89[1], 2) + pow(r_89[2], 2))
        nolm_r_910 = math.sqrt(pow(r_910[0], 2) + pow(r_910[1], 2) + pow(r_910[2], 2))
        den = nolm_r_89 * nolm_r_910
        radian = math.acos(num / den)
        # LElbowRollは-をつける
        angle = -math.degrees(radian)
        # print("LElbowRoll : {}".format(angle))
        return angle

        
    def convertVec(self, r2, r12, r16):
        # x, y, zを求める
        x_den = math.sqrt(pow(r16[0]-r12[0], 2) + pow(r16[1]-r12[1], 2) + pow(r16[2]-r12[2], 2))
        xx = (r16[0]-r12[0]) / x_den
        xy = (r16[1]-r12[1]) / x_den
        xz = (r16[2]-r12[2]) / x_den
        x = [xx, xy, xz]
        
        r16_12 = np.array([r16[0]-r12[0], r16[1]-r12[1], r16[2]-r12[2]])
        r16_2  = np.array([r16[0]- r2[0], r16[1] -r2[1], r16[2] -r2[2]])
        z_num = np.cross(r16_12, r16_2)
        z_den = math.sqrt(pow(z_num[0], 2) + pow(z_num[1], 2) + pow(z_num[2], 2))
        z = [z_num[0]/z_den, z_num[1]/z_den, z_num[2]/z_den]
        
        y = np.cross(np.array(z), np.array(x))
        return y, z

    def nao_move(self, angle):
        self.RSPMotor.setPosition(angle["RShoulderPitch"])
        self.RSRMotor.setPosition(angle["RShoulderRoll"])
        self.LSPMotor.setPosition(angle["LShoulderPitch"])
        self.LSRMotor.setPosition(angle["LShoulderRoll"])
        self.REYMotor.setPosition(angle["RElbowYaw"])
        self.RERMotor.setPosition(angle["RElbowRoll"])
        self.LEYMotor.setPosition(angle["LElbowYaw"])
        self.LERMotor.setPosition(angle["LElbowRoll"])

  
    def mirroring(self):
        timestep = int(self.robot.getBasicTimeStep())
        clientsock = self.tcpip_config()
        while self.robot.step(timestep) != -1:
            try:
                joint_angle = {}
                # 受け取ったmessage -> byteで来るので文字列に
                rcvmsg = str(clientsock.recv(1024))
                # e以下はいらないデータ
                cut_raw_string = rcvmsg.split("e")[0]
                # 先頭の文字が本来1なので置き換える, 各データの間に=を挟んでいるのでそれで区切る
                kinect_raw_list = ("1" + cut_raw_string[3:]).split("=")
                kinect_raw_list[0] = kinect_raw_list[0].replace("x00", "")
                # データ受け取れないと終了
                if rcvmsg == '':
                  break
                # 上半身の3次元座標が取得できているときのみRobotを動作
                if len(kinect_raw_list) == 16:
                    coordination = {}
                    # 文字列で座標を受け取るので,それぞれの関節毎にリスト化する
                    for raw_data_set in kinect_raw_list:
                        raw_data = raw_data_set.split(",")
                        coordination[int(raw_data[0])] = [float(raw_data[1]),\
                        float(raw_data[2]), float(raw_data[3])]
                    
                    y, z = self.convertVec(coordination[2], coordination[12], coordination[16])

                    # Mirroringは相手の右手に対して左手で模倣するので、人の右手の座標を左手の関節角に変換
                    LShoulderPitch = self.angleLShoulderPitch(coordination[8][0], coordination[8][1],\
                    coordination[8][2], coordination[9][0], coordination[9][1], coordination[9][2])
                    LShoulderRoll = self.angleLShoulderRoll(coordination[8][0], coordination[8][1],\
                    coordination[8][2], coordination[9][0], coordination[9][1], coordination[9][2], y, z)
                    
                    RShoulderPitch = self.angleRShoulderPitch(coordination[4][0], coordination[4][1],\
                    coordination[4][2], coordination[5][0], coordination[5][1], coordination[5][2])
                    RShoulderRoll = self.angleRShoulderRoll(coordination[4][0], coordination[4][1], \
                    coordination[4][2], coordination[5][0], coordination[5][1], coordination[5][2], y, z)
                    
                    # 8, 9, 10(shoulder, elbow, wrist)の座標, RshoulderRoll, RshoulderPitch, zが必要
                    LElbowYaw = self.angleLElbowYaw(coordination[8][0], coordination[8][1],coordination[8][2],\
                    coordination[9][0], coordination[9][1],coordination[9][2], coordination[10][0], coordination[10][1],\
                    coordination[10][2], RShoulderRoll, RShoulderPitch, z)

                    LElbowRoll = self.angleLElbowRoll(coordination[8][0], coordination[8][1], coordination[8][2], coordination[9][0],\
                    coordination[9][1], coordination[9][2], coordination[10][0], coordination[10][1], coordination[10][2])
                    
                    RElbowYaw = self.angleRElbowYaw(coordination[4][0], coordination[4][1], coordination[4][2],\
                    coordination[5][0], coordination[5][1], coordination[5][2], coordination[6][0], coordination[6][1],\
                    coordination[6][2], LShoulderRoll, LShoulderPitch, z)
                    
                    RElbowRoll = self.angleRElbowRoll(coordination[4][0], coordination[4][1], coordination[4][2], coordination[5][0],\
                    coordination[5][1], coordination[5][2], coordination[6][0], coordination[6][1], coordination[6][2])
                    
                    
                    joint_angle["LShoulderPitch"] = round(math.radians(LShoulderPitch), 4)
                    joint_angle["LShoulderRoll"] = round(math.radians(LShoulderRoll), 4)
                    joint_angle["RShoulderPitch"] = round(math.radians(RShoulderPitch), 4)
                    joint_angle["RShoulderRoll"] = round(math.radians(RShoulderRoll), 4)
                    joint_angle["LElbowYaw"] = round(math.radians(LElbowYaw), 4)
                    joint_angle["LElbowRoll"] = round(math.radians(LElbowRoll), 4)
                    joint_angle["RElbowYaw"] = round(math.radians(RElbowYaw), 4)
                    joint_angle["RElbowRoll"] = round(math.radians(RElbowRoll), 4)
                    
                    self.nao_move(joint_angle)
                    
                # 送信するmessage
                s_msg = b'ok'
                clientsock.sendall(s_msg)
                
            except Exception as e:
                print("Exception! : " + e)
                clientsock.close()
                
if __name__ == "__main__":
    nao = naoMirroring()
    nao.mirroring()