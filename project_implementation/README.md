HOW TO CHANGE BEHAVIOUR:

In file project_implementation.ino:
	
	Search for the line with " if(Thrust > 0 ){ ". There are three available triggering strategies that can be used. trig_func1 implements the analytical
	triggering strategy, trig_func2 implements the linearised version and finally trig_func3 implements an heuristic one.
	
	If trig_func2 is selected, uncomment the lines:
	//compute_lin_torques(q_1,q_2,&q_k,&lin_T[0],&lin_T[1],&lin_T[2],&lin_T[3],&lin_T[4]); // UNCOMMENT IF TRIG2 IS ENABLED
          /*q_k = mult_quat(q_1.conjugate(),q_2);
          q_k.sign_l();
          omega_k = omega;
          v_dot_k = - (omega_k * D_k) * omega_k ;*/
          
        If you want to record a log in the internal flash memory, uncomment:
        
        /* if(Roll > 10){
        Log_Write_torques(10000*tau.x,10000*tau.y,10000*tau.z);
        Log_Write_Attitude();
        Log_Write_Data(1, omega.z);
        Log_Write_Data(3,v_dot_k);
        Log_Write_Data(4,alpha);
        Log_Write_Data(5,debug);
        Log_Write_Data(6,T_star.x);
        if(trigger)
          Log_Write_Data(2,1);
        else
          Log_Write_Data(2,0);
      }*/
      
      with the condition in the if clause matching the conditions that trigger the log.
      
In the file saturating_controller.h:
	you can define all the parameters described in the report of this project.
	Most importantly, the c_T constant has direct impact in the quality of the time responde. Too low and the controller 
	will be unstable, too high and it will underperform (or not act at all). The ideal case would have a well estimated
c_T and changes only in the torque_multiplier constant. This would allow more control authority over the quad.
