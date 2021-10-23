
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <iostream> 

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>

using namespace std;
using namespace webots;


 



int main() {
	Motor *motors[4];//锟斤拷锟斤拷锟酵硷拷锟教讹拷要锟斤拷webots锟斤拷锟斤拷锟斤拷锟斤拷
	char wheels_names[4][8] = { "motor1","motor2","motor3","motor4" };//锟斤拷锟节凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫碉拷锟斤拷锟斤拷
	double speed1[4];
	double speed2[4];
	double velocity = 10;
	//锟斤拷锟斤拷锟斤拷一锟斤拷小锟斤拷锟今，碉拷锟侥革拷锟斤拷锟接帮拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷转锟斤拷时锟津，筹拷锟接匡拷锟斤拷锟斤拷锟斤拷前锟斤拷锟斤拷锟揭ｏ拷转圈
	//斜锟斤拷锟斤拷锟斤拷前锟斤拷+锟斤拷锟斤拷  锟斤拷锟斤拷锟斤拷同时锟斤拷
	double speed_forward[4] = { velocity ,velocity ,velocity ,velocity };
	double speed_backward[4] = { -velocity ,-velocity ,-velocity ,-velocity };
	double speed_leftward[4] = { velocity ,-velocity ,velocity ,-velocity };
	double speed_rightward[4] = { -velocity ,velocity ,-velocity ,velocity };
	double speed_go_left[4] = { velocity , 0 ,velocity ,0 };
	double speed_go_right[4] = { 0 ,velocity , 0,velocity };
	double speed_back_left[4] = { 0 ,-velocity ,0 ,-velocity };
	double speed_back_right[4] = { -velocity ,0 ,-velocity ,0 };
	double speed_leftCircle[4] = { velocity ,-velocity ,-velocity ,velocity };
	double speed_rightCircle[4] = { -velocity ,velocity ,velocity ,-velocity };

	webots::Keyboard keyboard;
	keyboard.enable(1);//锟斤拷锟叫硷拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷频锟斤拷锟斤拷1ms锟斤拷取一锟斤拷

	Robot *robot = new Robot();//使锟斤拷webots锟侥伙拷锟斤拷锟斤拷锟斤拷锟斤拷
	//锟斤拷始锟斤拷
	for (int i = 0; i < 4; i++)
	{
		motors[i] = robot->getMotor(wheels_names[i]);//锟斤拷锟斤拷锟斤拷锟节凤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫碉拷锟斤拷锟街伙拷取锟斤拷锟斤拷
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);
		speed1[i] = 0;
		speed2[i] = 0;
	}


	int timeStep = (int)robot->getBasicTimeStep();//锟斤拷取锟斤拷锟斤拷webots锟斤拷锟斤拷一帧锟斤拷时锟斤拷
	cout << timeStep << endl;

	Camera *camera;
	camera = robot->getCamera("camera");
	camera->enable(timeStep); // add Camera

	while (robot->step(timeStep) != -1) //锟斤拷锟斤拷锟斤拷锟斤拷一帧
	{



		//锟斤拷取锟斤拷锟斤拷锟斤拷锟诫，锟斤拷锟斤拷写锟斤拷锟皆伙拷锟斤拷同时锟斤拷锟铰的帮拷锟斤拷锟斤拷锟斤拷锟斤拷支锟斤拷7锟斤拷锟斤拷
		int keyValue1 = keyboard.getKey();
		int keyValue2 = keyboard.getKey();
		cout << keyValue1 << ":" << keyValue2 << endl;

		//锟斤拷锟捷帮拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷么锟斤拷转锟斤拷
		if (keyValue1 == 'W')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_forward[i];
			}
		}
		else if (keyValue1 == 'S')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_backward[i];
			}
		}
		else if (keyValue1 == 'A')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_leftward[i];
			}
		}
		else if (keyValue1 == 'D')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_rightward[i];
			}
		}
		else if (keyValue1 == 'Q')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_leftCircle[i];
			}
		}
		else if (keyValue1 == 'E')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_rightCircle[i];
			}
		}		
		else if (keyValue1 == '1')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_back_left[i];
			}
		}
		else if (keyValue1 == '3')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_back_right[i];
			}
		}
		else if (keyValue1 == '7')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_go_left[i];
			}
		}
		else if (keyValue1 == '9')
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = speed_go_right[i];
			}
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				speed1[i] = 0;
			}
		}




		if (keyValue2 == 'W')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_forward[i];
			}
		}
		else if (keyValue2 == 'S')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_backward[i];
			}
		}
		else if (keyValue2 == 'A')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_leftward[i];
			}
		}
		else if (keyValue2 == 'D')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_rightward[i];
			}
		}
		else if (keyValue2 == '1')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_back_left[i];
			}
		}
		else if (keyValue2 == '3')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_back_right[i];
			}
		}
		else if (keyValue2 == '7')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_go_left[i];
			}
		}
		else if (keyValue2 == '9')
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = speed_go_right[i];
			}
		}
		else 
		{
			for (int i = 0; i < 4; i++)
			{
				speed2[i] = 0;
			}
		}

		//锟矫碉拷锟斤拷执锟斤拷
		for (int i = 0; i < 4; i++)
		{
			motors[i]->setVelocity(speed1[i] + speed2[i]);
		}

		//wb_motor_set_velocity(wheels[0],right_speed);
	}


	return 0;
}
