#include <iostream>
#include <sstream>
#include <fstream>
#include "NtKinect.h"
#include <Kinect.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <Windows.h>

#define PORT 9876

using namespace std;

void doJob() {
	NtKinect kinect;
	int i = 0;
	clock_t start = clock();
	// ----TCP/IP通信のための設定----
	// IP アドレス，ポート番号，ソケット，sockaddr_in 構造体
	int dstSocket;
	struct sockaddr_in dstAddr;
	// 各種パラメータ
	char buffer[1024];
	char res_buf[1024];
	string send_buf;
	// Windows の場合
	WSADATA data;
	WSAStartup(MAKEWORD(2, 0), &data);
	// 相手先アドレスの入力と送る文字の入力
	const char *destination = "163.221.38.217";

	// sockaddr_in 構造体のセット
	memset(&dstAddr, 0, sizeof(dstAddr));
	dstAddr.sin_port = htons(PORT);
	dstAddr.sin_family = AF_INET;
	//IP アドレスの変換
	inet_pton(dstAddr.sin_family, destination, &dstAddr.sin_addr.S_un.S_addr);
	// ソケットの生成
	dstSocket = socket(AF_INET, SOCK_STREAM, 0);

	//接続
	if (connect(dstSocket, (struct sockaddr *) &dstAddr, sizeof(dstAddr))) {
		printf("%s cannnot be connected.\n", destination);
	}
	printf("%s is connected.\n", destination);

	// ------------------

	while (1) {
		kinect.setRGB();
		kinect.setSkeleton();
		// kinectに映った人の骨格情報[list]を一人ずつloop
		for (auto person : kinect.skeleton) {
			// std::cout << person.size();
			// 毎フレームごとにテキストファイルを保存
			i++;
			if (i % 20 == 0) {
				std::string path = "/Users/P4/workspace/mirroring/video_experiment/honda/" + std::to_string(i) + ".txt";
				ofstream outputfile(path);
				// 1人の関節情報をloop (jointtypeは25個)
				for (auto joint : person) {
					int jt = joint.JointType;
					// 上半身の関節を取得 , fps5にする
					if (((jt == 1) || (jt == 2) || (jt == 3) || (jt == 4) || (jt == 5) || (jt == 6) || (jt == 7) || (jt == 8) ||
						(jt == 9) || (jt == 10) || (jt == 11) || (jt == 12) || (jt == 16) || (jt == 22) || (jt == 23))) {
						outputfile << joint.JointType;
						outputfile << ",";
						outputfile << joint.Position.X;
						outputfile << ",";
						outputfile << joint.Position.Y;
						outputfile << ",";
						outputfile << joint.Position.Z << std::endl;
						snprintf(buffer, 1024, "%d,%f,%f,%f=", joint.JointType, joint.Position.X, joint.Position.Y, joint.Position.Z);
						send_buf += buffer;
					}
					if (jt == 24) {
						snprintf(buffer, 1024, "%d,%f,%f,%fe\n", joint.JointType, joint.Position.X, joint.Position.Y, joint.Position.Z);
						send_buf += buffer;
					}
					// 画像を表示
					if (joint.TrackingState == TrackingState_NotTracked) continue;
					ColorSpacePoint cp;
					kinect.coordinateMapper->MapCameraPointToColorSpace(joint.Position, &cp);
					cv::rectangle(kinect.rgbImage, cv::Rect((int)cp.X - 5, (int)cp.Y - 5, 20, 20), cv::Scalar(0, 100, 200), 2);
				}
				clock_t end = clock();
				const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
				outputfile << time;
				std::cout << time << std::endl;
			}
		}
		// ここでclientの処理を挟む : sever側にデータを送信する
		send(dstSocket, send_buf
			.c_str(), 1024, 0);
		//パケットの受信
		recv(dstSocket, res_buf, 1024, 0);
		printf(">>%s\n", res_buf);

		std::cout << send_buf;
		std::cout << "----------------\n";
		send_buf = "";
		cv::resize(kinect.rgbImage, kinect.rgbImage, cv::Size(), 0.4, 0.4);
		cv::imshow("rgb", kinect.rgbImage);
		//Sleep(1);
		// key入力 qが入力されたら終了
		auto key = cv::waitKey(1);
		if (key == 'q') break;
	}
	// Windows でのソケットの終了
	closesocket(dstSocket);
	WSACleanup();
	cv::destroyAllWindows();
}

int main(int argc, char** argv) {
	try {
		doJob();
	}
	catch (exception &ex) {
		cout << ex.what() << endl;
		string s;
		cin >> s;
	}
	return 0;
}