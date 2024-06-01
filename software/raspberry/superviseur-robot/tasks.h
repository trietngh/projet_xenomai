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

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    
    ComMonitor monitor;
    ComRobot robot;
    int move = MESSAGE_ROBOT_STOP;

    /*Added Shared Objects*/
    Camera camera_supervisor;
    Arena current_arena;
    Position current_position; 
    Img* current_image;

    /*Added Shared Variables*/
    /*
     * positionRequired = 1 : if robot position request
     *                    0 : if stop robot position request 
     */
    int positionRequired = 0;
    /*
     * robotStarted = 1 : if robot started
     *                0 : if robot stopped
     */
    int robotStarted = 0;
    /*
     *cameraStarted = 1 : if camera started
     *                0 : if camera stopped
     */
    int cameraStarted = 0;
    /*
     * areneRequired = 1 : if find arena request
     *                 0 : if stop find arena request
     */
    int areneRequired = 0;
    /*
     * confirmArena = 1 : if arena image confirmed
     *                0 : if arena image not confirmed
     */
    int confirmArena = 0;

    
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;

    /**
     * Added tasks
     */
    /* Task to check the battery level */
    RT_TASK th_batterycheck;
    /* Task to send image to monitor */
    RT_TASK th_sendImageToMon;
    /* Task to send robot's position to monitor */
    RT_TASK th_sendPositionToMon;
    /* Task to check on the connexion with the robot */
    RT_TASK th_checkRobotConnection;

    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;
    /*
     * Added mutexes
     */
    /* Mutex for the object camera */
    RT_MUTEX mutex_camera;
    /* Mutex for the camera started */
    RT_MUTEX mutex_cameraStarted;
    /* Mutex for the find arena request */
    RT_MUTEX mutex_areneRequired;
    /* Mutex for the object arena */
    RT_MUTEX mutex_arena;
    /* Mutex for the confirm arena image */
    RT_MUTEX mutex_confirmArena;
    /* Mutex for the send position request */
    RT_MUTEX mutex_positionRequired;
    /* Mutex for the object position */
    RT_MUTEX mutex_current_position;
    /* Mutex for the grabed object image*/
    RT_MUTEX mutex_current_image;

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;
    /*
     * Added semaphores
     */
    /* Semaphore for the battery check request */
    RT_SEM sem_checkBatt;
    /* Semaphore for the start camera */
    RT_SEM sem_camON;

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
     /**
     * @brief Thread handling control of the robot.
     */
    void ReadBattLevel(void *arg);
    
    /**
     * @brief Thread handling the grabing and the sending of the image to the monitor.
     */
    void SendImageToMon(void *arg);

    /**
     * @brief Thread handling the calculating and the sending of the robot's position to the monitor.
     */
    void SendPositionToMon(void);

    /**
     * @brief Thread handling the checking of the connexion with the robot.
     */
    void CheckRobotConnection(void);

    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);
    
    /**********************************************************************/
    /* Camera functions                                                   */
    /**********************************************************************/
    /**
     * @brief Function to open the camera.
     */
    void StartCamera(void);
    /**
     * @brief Function to close the camera.
     */
    void CloseCamera(void);
    /**
     * @brief Function to find the arena in the current_image.
     */
    void StartFindArena(void); 
};

#endif // __TASKS_H__ 

