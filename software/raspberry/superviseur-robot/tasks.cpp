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

/**
 * @todo Remains to be done:
 * - Result the issue causing the messageQueue to crash because it can not allocate memory
 * - Protect the access to shared data when READ
 * - Change findArena to a separate thread
 * - Protect the access to the queue with a mutex 
 */

/**
 * @note Acquired skills:
 * Mutexes are used to protect the access to shared data.
 * Semaphores are used to synchronize the tasks.
 * Set priority of the tasks to avoid deadlocks.
 * Use of the periodic task to send the battery level, check the connection and send the image to the monitor.
 * Use of the activation task to start the robot.
 * Use of the message queue to send messages and images to the monitor.
 */



/* Declaration of the priority level of the tasks */
/*
 * @note The current arrangement of priorities is functionnal but not optimal.
 * The sendImageToMon task have a higher lantency than expected, it should be more prioritary. 
 */
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 29
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 27
#define PRIORITY_TSTARTROBOT 28
#define PRIORITY_TBATTCHECK 23
#define PRIORITY_TSENDIMAGETOMON 25
#define PRIORITY_TSENDPOSITIONTOMON 19
#define PRIORITY_TCHECKROBOTCONNECTION 31


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
    if (err = rt_mutex_create(&mutex_areneRequired, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_confirmArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_positionRequired, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_current_position, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_current_image, NULL)) {
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
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_checkBatt, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_camON, NULL, 0, S_FIFO)) {
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
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_batterycheck, "th_batterycheck", 0, PRIORITY_TBATTCHECK, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_create(&th_sendImageToMon, "th_sendImageToMon", 0, PRIORITY_TSENDIMAGETOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendPositionToMon, "th_sendPositionToMon", 0, PRIORITY_TSENDPOSITIONTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_create(&th_checkRobotConnection, "th_checkRobotConnection", 0, PRIORITY_TCHECKROBOTCONNECTION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*100, Q_UNLIMITED, Q_FIFO)) < 0) {
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
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }    
    if (err = rt_task_start(&th_batterycheck, (void(*)(void*)) & Tasks::ReadBattLevel, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendImageToMon, (void(*)(void*)) & Tasks::SendImageToMon, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendPositionToMon, (void(*)(void*)) & Tasks::SendPositionToMon, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_checkRobotConnection, (void(*)(void*)) & Tasks::CheckRobotConnection, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
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
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
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
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
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
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);   
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_broadcast(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET))
        {
            rt_sem_v(&sem_checkBatt);   //Increase the semaphore to start the battery check
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN))
        {            
            StartCamera();     //Call function to open the camera
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE))
        {
            /* Set flag to 0 to stop all tasks that use the camera */
            rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
            cameraStarted = 0;
            rt_mutex_release(&mutex_cameraStarted);

            CloseCamera();      //Call function to close the camera
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA))
        {        
            /* Set flag to change the sendImagetoMon task into findArena state */  
            rt_mutex_acquire(&mutex_areneRequired, TM_INFINITE);
            areneRequired = 1;
            rt_mutex_release(&mutex_areneRequired);
            /* Start finding the arena */
            /**
             * @note This function could take too long to execute, it should be in a separate thread 
             */
            StartFindArena();
        }
        else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM))
        {
            cout << "CONFIRM : Arena Image" << endl << flush;
            rt_mutex_acquire(&mutex_confirmArena, TM_INFINITE);
            confirmArena = 1;
            rt_mutex_release(&mutex_confirmArena);
            /* Set flag to change the sendImagetoMon task back into peiodic sending state */
            rt_mutex_acquire(&mutex_areneRequired, TM_INFINITE);
            areneRequired = 0;
            rt_mutex_release(&mutex_areneRequired);

            rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
            cameraStarted = 1;
            rt_mutex_release(&mutex_cameraStarted);

        }
        else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM))
        {
            cout << "REJECT : Arena Image" << endl << flush;
            rt_mutex_acquire(&mutex_confirmArena, TM_INFINITE);
            confirmArena = 0;
            rt_mutex_release(&mutex_confirmArena);
            /* Set flag to change the sendImagetoMon task back into peiodic sending state */
            rt_mutex_acquire(&mutex_areneRequired, TM_INFINITE);
            areneRequired = 0;
            rt_mutex_release(&mutex_areneRequired);

            rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
            cameraStarted = 1;
            rt_mutex_release(&mutex_cameraStarted);
        }
        else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START))
        {
            rt_mutex_acquire(&mutex_positionRequired, TM_INFINITE);
            positionRequired = 1;
            rt_mutex_release(&mutex_positionRequired);
        }

        else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP))
        {
            rt_mutex_acquire(&mutex_positionRequired, TM_INFINITE);
            positionRequired = 0;
            rt_mutex_release(&mutex_positionRequired);
        }

        delete(msgRcv); // mus be deleted manually, no consumer
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
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
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
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
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
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    int oldMove = -1;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
        /* Only print when a new order is received */
        if (oldMove != cpMove){
            cout << "Periodic movement update";
            cout << " move: " << cpMove;
            cout << endl << flush;
            oldMove = cpMove;
        }       
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
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
    }
    return msg;
}

/**
 * @brief Thread handling the reading of the battery level.
 */
void Tasks::ReadBattLevel(void *arg){
        Message * msgSend;
        bool enableBatteryCheck = 0;
        RT_SEM_INFO info_semCheckBatt;
        
        rt_sem_p(&sem_barrier, TM_INFINITE);
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        rt_sem_p(&sem_checkBatt, TM_INFINITE);
        cout << "Activate Battery Check" << endl << flush;
        enableBatteryCheck = 1;
        rt_task_set_periodic(NULL, TM_NOW, 500000000);
        while (1) {
            rt_task_wait_period(NULL);
            if(enableBatteryCheck == 1){
                cout << "Periodic Battery Level update" << endl << flush;
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgSend = robot.Write(robot.GetBattery());
                rt_mutex_release(&mutex_robot);
                WriteInQueue(&q_messageToMon, msgSend);
            }
            /* Try to make task stop sending the battery state when unchecking the box in monitor
             * But currently it doesn't work.
             * Can be deleted.
             */
            rt_sem_inquire(&sem_checkBatt, &info_semCheckBatt);
            if (info_semCheckBatt.count != 0 && enableBatteryCheck == 1){
                rt_sem_p(&sem_checkBatt, TM_INFINITE);
                enableBatteryCheck = 0;  
            } else if (info_semCheckBatt.count != 0 && enableBatteryCheck == 0){
                rt_sem_p(&sem_checkBatt, TM_INFINITE);
                enableBatteryCheck = 1;  
            }
        }

}

/**
 * @brief Thread handling the grabing and the sending of the image to the monitor.
 */
void Tasks::StartCamera(void){
    bool status;
    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
    status = camera_supervisor.Open();
    rt_mutex_release(&mutex_camera);

    if(status == false){
        cerr << "Open Camera fail" << endl << flush;
        exit(EXIT_FAILURE);
    }

    /* Set the cameraStarted flag to 1 when the camera is open */
    rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
    cameraStarted = 1;
    rt_mutex_release(&mutex_cameraStarted);
    
    rt_sem_v(&sem_camON);
}

void Tasks::SendImageToMon(void *arg)
{
    MessageImg * msgImgSend;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_camON, TM_INFINITE);
    
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        /*Check if the camera is open and the cameraStarted flag is set and this is not required to find the arena*/
        if(cameraStarted == 1 && areneRequired == 0 && camera_supervisor.IsOpen())
        {
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            Img new_image = camera_supervisor.Grab();
            rt_mutex_release(&mutex_camera);
            
            /*If the confirmArena flag is set, draw the arena in the image*/
            if(confirmArena == 1)
            {
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                new_image.DrawArena(current_arena);
                rt_mutex_release(&mutex_arena);
            }

            /*If the positionRequired flag is set, draw the robot in the image*/
            if(positionRequired == 1)
            {
                rt_mutex_acquire(&mutex_current_position, TM_INFINITE);
                new_image.DrawRobot(current_position);
                rt_mutex_release(&mutex_current_position);
            }
            
            rt_mutex_acquire(&mutex_current_image, TM_INFINITE);
            current_image = new_image.Copy();       //Store image to findArena if nedded
            rt_mutex_release(&mutex_current_image);
            
            cout << "Sending Image" << endl << flush;
            /**
             * @note j'ai remarqué que si WriteInQueue supprime le message et l'image dedans,
             *       mon current_image n'a jamais de valeur...
             * 
             *       Si c'était le cas, la fonction sendPositionToMon ne fonctionnerait pas.
             *       Mais c'est (peut-être) pas le cas.
             * 
             *       Donc il est probable que WriteInQueue ne supprime pas l'image...
             *       A tester...
             */
            msgImgSend = new MessageImg(MESSAGE_CAM_IMAGE, current_image);
            
            WriteInQueue(&q_messageToMon, msgImgSend);
        }
    }
}

/**
 * @brief Function to close the camera, no task should use the camera after this function
 */
void Tasks::CloseCamera(void)
{
    cout << "Closed Camera" << endl << flush;
    
    camera_supervisor.Close();        
    Message * msgSend;
    msgSend = new Message(MESSAGE_ANSWER_ACK);
    
    WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
}

/**
 * @brief Function to find the arena in the image, store in current_Arena and send the image to the monitor
 */
void Tasks::StartFindArena(void)
{
    MessageImg * msgImgSend;
    /*Check if the camera is open and the cameraStarted flag is set*/
    if(cameraStarted == 1 && camera_supervisor.IsOpen()){
        
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted = 0;
        rt_mutex_release(&mutex_cameraStarted);
        cout << "SUCESS : Finding the arena" << endl << flush;
        
        /*Grab the image*/
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        Img new_image = camera_supervisor.Grab();
        rt_mutex_release(&mutex_camera);

        /*Find the arena in the image*/
        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
        current_arena = new_image.SearchArena();
        /*Draw the arena in the image*/
        new_image.DrawArena(current_arena);
        rt_mutex_release(&mutex_arena);
        
        /*Send the image to the monitor*/
        Img * ptr_img = new Img(new_image);
        msgImgSend = new MessageImg(MESSAGE_CAM_IMAGE, ptr_img);
        WriteInQueue(&q_messageToMon, msgImgSend);
    }
    else if(cameraStarted == 0){
        cout << "FAIL : Camera closed, cannot find the arena" << endl << flush;
    }
}

/**
 * @brief Thread sending the position of the robot to the monitor
 */
void Tasks::SendPositionToMon(void){
    MessagePosition * msgPosSend;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        if(positionRequired == 1)
        {
            /*Protect the access to the shared data*/
            rt_mutex_acquire(&mutex_current_image, TM_INFINITE);
            rt_mutex_acquire(&mutex_current_position, TM_INFINITE);
            /*Search the robot position in the image*/
            current_position = (current_image->SearchRobot(current_arena)).front();
            
            /*If the robot is not found, set the position to -1*/
            if(current_position.robotId == 0)
            {
                current_position.center.x= -1.0;
                current_position.center.y= -1.0;
                current_position.angle = -1.0;
                current_position.direction.x= -1.0;
                current_position.direction.y= -1.0;
            }
            rt_mutex_release(&mutex_current_image);
            rt_mutex_release(&mutex_current_position);
    
            /*Send the position to the monitor*/
            msgPosSend = new MessagePosition(MESSAGE_CAM_POSITION, current_position);
            WriteInQueue(&q_messageToMon, msgPosSend);
        }
    }
}

/**
 * @brief Thread checking the connection with the robot every 100ms
 */

void Tasks::CheckRobotConnection(void)
{
    Message * msgSend;
    int counter = 0;
    //int rs;
    //bool LostConnexion = False;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_startRobot, TM_INFINITE);

    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        if (robotStarted == 1)
        {
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.Ping());
            rt_mutex_release(&mutex_robot);

            /*if the received message is timeout, increment the counter*/
            if (*msgSend == MESSAGE_ANSWER_ROBOT_TIMEOUT)
            {
                counter++;
            }
            else 
            {
                counter = 0;
            }         
        }
        /*
        if counter reach 10, the robot is considered disconnected
        */
        if(counter == 10){
            /*close the communication with robot*/
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            rt_mutex_release(&mutex_robot);

            /*set the robot stopped status*/
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            /*reset counter*/
            counter = 0;
        }
        
    }
    
}