//#include <motor_work.h>
//#include <lidar.h>
//#include <ssbot_control.h>
//#include <joystick.h>
//
//extern bool open_can_success;
//extern int ARM_STATE;
//extern motor M;
//
//void main()
//{
//	if (true)
//	{
//		if (M.init(FIRST_MOTOR_CAN, MODE_POSITION_S, 1))
//		//if(true)
//		{
//			double angle_goal = -180;
//			M.setAngle(angle_goal, FIRST_MOTOR_CAN);
//			double angle_now;
//			while (true)
//			{
//				angle_now = M.getAngle(FIRST_MOTOR_CAN);
//				if (abs(angle_now - angle_goal) < 0.01)
//				{
//					break;
//				}
//			}
//		}
//		arm_disable();
//	}
//
//	//thread t5(joystick1_listen);
//	//thread t6(joystick2_listen);
//
//	//t5.join();
//	//t6.join();
//
//	return;
//}