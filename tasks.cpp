/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TCLOSECOMROBOT 20
#define PRIORITY_TBATTERY 19
#define PRIORITY_TRELOADWD 24
#define PRIORITY_SENDIMG 20
#define PRIORITY_GETARENA 21
#define ROBOT_ID 2

#define P_100MS 100000000
#define P_1S 1000000000
/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cameraStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_askArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_computePosition, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_robotStarted, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_restartServer, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_wd, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_askArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_closeComRobot, "th_closeComRobot", 0, PRIORITY_TCLOSECOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_create(&th_reloadWD, "th_reloadWD", 0, PRIORITY_TRELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_create(&th_sendImg, "th_sendImg", 0, PRIORITY_SENDIMG, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_create(&th_getArena, "th_getArena", 0, PRIORITY_GETARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeComRobot, (void(*)(void*)) & Tasks::CloseComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWD, (void(*)(void*)) & Tasks::ReloadWDTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if(err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::OpenCamera, this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_start(&th_closeCamera, (void(*)(void*)) & Tasks::CloseCamera, this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_start(&th_sendImg, (void(*)(void*)) & Tasks::SendImage, this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if(err = rt_task_start(&th_getArena, (void(*)(void*)) & Tasks::GetArena, this)){
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    monitor.Close();
    rt_mutex_release(&mutex_monitor);
    
    rt_sem_v(&sem_closeComRobot);
    rt_sem_v(&sem_closeCamera);
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    while(1){
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        cout << "open monitor" << endl << flush;
        status = monitor.Open(SERVER_PORT);
        rt_mutex_release(&mutex_monitor);

        cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

        if (status < 0) throw std::runtime_error {
            "Unable to start server on port " + std::to_string(SERVER_PORT)
        };
        monitor.AcceptClient(); // Wait the monitor client
        cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
        serverStarted = true;
        rt_sem_broadcast(&sem_serverOk);

        rt_sem_p(&sem_restartServer, TM_INFINITE);

        cout << "Restarting Server.." << endl << flush;
    }

    
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    while(1){
        cout << "wait on sendToMon" << endl << flush;
        rt_sem_p(&sem_serverOk, TM_INFINITE);

        while (serverStarted) {
            cout << "wait msg to send" << endl << flush;
            msg = ReadInQueue(&q_messageToMon);
            cout << "Send msg to mon: " << msg->ToString() << endl << flush;
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msg); // The message is deleted with the Write
            rt_mutex_release(&mutex_monitor);
        }
    }
    
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    
    while(1){
        cout << "wait on ReceivFrom" << endl << flush;
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        cout << "Received message from monitor activated" << endl << flush;

        while (serverStarted) {
            cout << "inside read" <<endl << flush ;
            msgRcv = monitor.Read();
            cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

            if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {               
                this->Stop();
                cout << "Connection to Monitor is lost" << endl << flush;
                serverStarted = false;
                rt_sem_v(&sem_restartServer);

            } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
                rt_sem_v(&sem_openComRobot);
               
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
                mode = MESSAGE_ROBOT_START_WITHOUT_WD;
                rt_sem_broadcast(&sem_startRobot);
            }else if(msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)){
                mode = MESSAGE_ROBOT_START_WITH_WD;
                rt_sem_broadcast(&sem_startRobot);
                
            } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)){
                rt_sem_v(&sem_openCamera);
            } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
                rt_sem_v(&sem_closeCamera);
                
            } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
                rt_mutex_acquire(&mutex_askArena, TM_INFINITE);
                askArena = true;
                rt_mutex_release(&mutex_askArena);        
                rt_sem_v(&sem_askArena);
            } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
                rt_mutex_acquire(&mutex_askArena, TM_INFINITE);
                askArena = false;
                rt_mutex_release(&mutex_askArena);
            } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
                arena = 0;
                rt_mutex_acquire(&mutex_askArena, TM_INFINITE);
                askArena = false;
                rt_mutex_release(&mutex_askArena);
            } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
                
                rt_mutex_acquire(&mutex_computePosition, TM_INFINITE);
                computePosition = true;
                rt_mutex_release(&mutex_computePosition);
                
            } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
                
                rt_mutex_acquire(&mutex_computePosition, TM_INFINITE);
                computePosition = false;
                rt_mutex_release(&mutex_computePosition);
                
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
                
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_RESET)){
                SendReset();

                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = MESSAGE_ROBOT_STOP;
                rt_mutex_release(&mutex_move);
            }
            delete(msgRcv); // mus be deleted manually, no consumer
        }
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        
        cout << "wait on ComRobot" << endl << flush;
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "inside ComRobot" << endl << flush;
        
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            comRobotStarted = true;
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread closing the communication with the robot.
 */
void Tasks::CloseComRobot(void *arg) {
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task closeComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        cout << "wait on CloseComRobot" << endl << flush;
        rt_sem_p(&sem_closeComRobot, TM_INFINITE);
        cout << "inside CloseComRobot" << endl << flush;
        if(comRobotStarted){
            cout << "Close serial com (";
            
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = MESSAGE_ROBOT_STOP;
            rt_mutex_release(&mutex_move);

            
            SendReset();
                   
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            status = robot.Close();
            rt_mutex_release(&mutex_robot);

            comRobotStarted = false;

            cout << status;
            cout << ")" << endl << flush;
        }  
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        Message * msgSend;
        
        cout << "wait on StartTask" << endl << flush;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        
        
        if(mode == MESSAGE_ROBOT_START_WITHOUT_WD){
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
        }else if(mode == MESSAGE_ROBOT_START_WITH_WD){
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_mutex_release(&mutex_robot);
        }
        
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            if(mode == MESSAGE_ROBOT_START_WITH_WD){
                rt_sem_v(&sem_wd);
            }
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            rt_sem_v(&sem_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while(1){
        cout << "wait on Move" << endl << flush;
        rt_sem_p(&sem_serverOk, TM_INFINITE);
    
        rt_task_set_periodic(NULL, TM_NOW, P_1S + 5*P_100MS);

        while (serverStarted) {
            
            Message * msgRcv;

            rt_task_wait_period(NULL);
            //cout << "Periodic movement update";
            
            rs = GetRobotStarted();
            
            cout << "rstarted = " << rs;
            if (rs == 1) {
                cout << "SendingMove" << endl << flush;
                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                cpMove = move;
                rt_mutex_release(&mutex_move);
                
                cout << " move: " << cpMove;
                
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgRcv = robot.Write(new Message((MessageID)cpMove));
                rt_mutex_release(&mutex_robot);
                
                CheckComRobot(msgRcv);
                
            }
            cout << endl << flush;
        }
        rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);
   }
     
}

/*
 @brief Thread to reload the watchdog if the corresponding mode in chosen
 */
void Tasks::ReloadWDTask(void *arg){
    Message * msgRcv;
    int rs;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/

    while(1){
        cout << "wait on reloadWD" << endl << flush;
        rt_sem_p(&sem_wd, TM_INFINITE);
        cout << "inside reloadWD" << endl << flush;
        
        rt_task_set_periodic(NULL, TM_NOW, P_1S);
        
        rs = GetRobotStarted();

        while(rs == 1){
            rt_task_wait_period(NULL);
            cout << "In RELOAD WD" << endl << flush;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgRcv = robot.Write(new Message(MESSAGE_ROBOT_RELOAD_WD));
            rt_mutex_release(&mutex_robot);
            
            CheckComRobot(msgRcv);
            
            rs = GetRobotStarted();
            
        }
        rt_task_set_periodic(NULL,TM_NOW, TM_INFINITE);
    }
}

/*
 @brief thread to get the battery level. 
 */
void Tasks::BatteryTask(void *arg){   
    Message * Lbatterie;   
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while(1){
        cout << "wait on Battery" << endl << flush;
        rt_sem_p(&sem_robotStarted, TM_INFINITE);
        cout << "inside Battery" << endl << flush;

        rt_task_set_periodic(NULL, TM_NOW, 2*P_1S);

        cout << "Battery check" <<endl;
        
        rs = GetRobotStarted();
        
        while(rs == 1){
            cout << "In the while" <<endl;
            rt_task_wait_period(NULL);
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);   
            Lbatterie = robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);      
            
            cout << "Msg battery : " << Lbatterie->ToString() <<endl;
            
            CheckComRobot(Lbatterie);
            
            if(!Lbatterie->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)){
                WriteInQueue(&q_messageToMon, Lbatterie);
            }
             
            rs = GetRobotStarted();
           
        }

        rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);
    }
}


 void Tasks::OpenCamera(void *arg){
     int status;

     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
     // Synchronization barrier (waiting that all tasks are starting)
     rt_sem_p(&sem_barrier, TM_INFINITE);
    
     /**************************************************************************************/
     /* The task starts here                                                               */
     /**************************************************************************************/
    
     while(1) {
         rt_sem_p(&sem_openCamera, TM_INFINITE);
        
         cout << "Open com camera(";
         
         rt_mutex_acquire(&mutex_camera, TM_INFINITE);
         status = camera.Open();
         rt_mutex_release(&mutex_camera);
         cout << status;
         cout << ")" << endl << flush;

         Message * msgSend;
         if (status < 0) {
             msgSend = new Message(MESSAGE_ANSWER_NACK);
         } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
            cameraStarted = true;
            rt_mutex_release(&mutex_cameraStarted);
         }
         WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
     }
 }
 
 void Tasks::CloseCamera(void *arg){
     bool cs;

     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
     // Synchronization barrier (waiting that all tasks are starting)
     rt_sem_p(&sem_barrier, TM_INFINITE);
    
     /**************************************************************************************/
     /* The task starts here                                                               */
     /**************************************************************************************/
    
     while(1) {
         rt_sem_p(&sem_closeCamera, TM_INFINITE);
        
         cs = GetCameraStarted();
         
         if(cs){
            rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
            cameraStarted = false;
            rt_mutex_release(&mutex_cameraStarted);
             
            cout << "Close com camera( OK )" << endl << flush;
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera.Close();
            rt_mutex_release(&mutex_camera);
            
            

            Message * msgSend = new Message(MESSAGE_ANSWER_ACK);
            
            WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
         }
        
     }
 }


 
 /**
 * @brief Thread handling control of the robot.
 */
void Tasks::SendImage(void *arg) {
    bool cs, aa, cp;
    int robotId = ROBOT_ID;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while(1){
        cout << "wait on SendImg" << endl << flush;
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        
        rt_task_set_periodic(NULL, TM_NOW, P_100MS);

        while (serverStarted) {
            
            Message * msgImg;
            Img *img;

            rt_task_wait_period(NULL);
            cout << "Periodic img update";
            
            cs = GetCameraStarted();
            
            rt_mutex_acquire(&mutex_askArena, TM_INFINITE);
            aa = askArena;
            rt_mutex_release(&mutex_askArena);
            
            if (cs && !aa) {
                cout << "SendingImg" << endl << flush;
                           
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                img =  new Img(camera.Grab());
                rt_mutex_release(&mutex_camera);
                
                rt_mutex_acquire(&mutex_computePosition, TM_INFINITE);
                cp = computePosition;
                rt_mutex_release(&mutex_computePosition);
                
                if(arena != 0){
                    img->DrawArena(*arena);
                }
                
                if(cp){
                    Message * msgPos;
                    *positionList = img->SearchRobot(*arena);
                    
                    Position pos;
                    pos.center = cv::Point2f(-1.0,-1.0);
                    
                    for (Position p: *positionList){
                        img->DrawRobot(p);
                        if(p.robotId = robotId){
                            pos = p;
                        }
                    }
                    msgPos = new MessagePosition(MESSAGE_CAM_POSITION, pos);
                    WriteInQueue(&q_messageToMon, msgPos);
                }
                
                msgImg = new MessageImg(MESSAGE_CAM_IMAGE,img);
                WriteInQueue(&q_messageToMon, msgImg);
            }
            cout << endl << flush;
        }
        rt_task_set_periodic(NULL, TM_NOW, TM_INFINITE);
   }
     
}

void Tasks::GetArena(void *arg){
    bool cs;
    Message * msgSend;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    while(1){
        rt_sem_p(&sem_askArena, TM_INFINITE);
        cs = GetCameraStarted();
        
        if(cs) {
            cout << "in ASK ARENA" << endl << flush;
            Img * img = new Img(camera.Grab());
            *arena = img->SearchArena();
            if(arena->IsEmpty()){
                msgSend = new Message(MESSAGE_ANSWER_NACK);
                arena = 0;
            }else{
                img->DrawArena(*arena);
                msgSend = new MessageImg(MESSAGE_CAM_IMAGE, img);
            }
            WriteInQueue(&q_messageToMon, msgSend);
        }else{
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
        }
        
    }
}
 
 
 
/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

/**
 * Check if the id of the given message corresponds to a timeout
 * If it is the case, increase a counter. Reset it to 0 otherwise
 * If the counter is greater than 3, send an error message to the monitor
 * and close the robot's communication 
 * @param message
 */
void Tasks::CheckComRobot(Message *message){
    if(message->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)){
        cptErr++;
        if(cptErr > 3){
            cout << "3 timeouts !!!!"<< endl << flush;
            cptErr = 0;
            
            Message * msgSend = new Message(MESSAGE_ANSWER_COM_ERROR);
            WriteInQueue(&q_messageToMon, msgSend);
            
            rt_sem_v(&sem_closeComRobot);
            
        }
    }else{
        cptErr = 0;
    }
}

/*
 @brief Send the message "reset" three times to the robot 
 * to avoid timeout or other error cases
 */
void Tasks::SendReset(){
    for(int i=0;i<3;i++){
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        robot.Write(robot.Reset());
        rt_mutex_release(&mutex_robot);
    }
}

int Tasks::GetRobotStarted(){
    rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
    int rs = robotStarted;
    rt_mutex_release(&mutex_robotStarted);
    return rs;
}

bool Tasks::GetCameraStarted(){
    rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
    bool cs = cameraStarted;
    rt_mutex_release(&mutex_cameraStarted);
    return cs;
}
