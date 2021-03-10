/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    int cptErr = 0;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            Message *  msgRcv;
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgRcv = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
            
            if(msgRcv->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)){
                cptErr++;
                if(cptErr > 3){
                    
                    cout << "Perte de communication" << endl << flush;
                    Message * msgSend = new Message(MESSAGE_MONITOR_LOST);
                    WriteInQueue(&q_messageToMon, msgSend);
                    
                    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                    robotStarted = 0;
                    rt_mutex_release(&mutex_robotStarted);
                    
                    rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                    robot.Close();
                    rt_mutex_release(&mutex_robot);
                }
            }else{
                cptErr = 0;
            }
        }
        cout << endl << flush;
    }
}
