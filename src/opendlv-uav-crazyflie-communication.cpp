/*
 * Copyright (C) 2018  Christian Berger
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

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <array>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include "crazyflieLinkCpp/Connection.h"
#include "PacketUtils.hpp"
#include "Crazyflie.h"

using namespace bitcraze::crazyflieLinkCpp;

#include <cmath>

float quantizeYaw(float yaw) {
    if (yaw <= 2.5f)
        return 1.25f;
    else {
        int bucket = static_cast<int>(std::ceil((yaw - 2.5f) / 2.5f));
        return bucket * 2.5f + 1.25f;
    }
}


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ) {
        std::cerr << "You should include the cid to start communicate in OD4Session" << std::endl;
        return retCode;
    }

    if ( (0 == commandlineArguments.count("radiouri")) ) {
        std::cerr << "You should include the radiouri to start communicate to crazyflie" << std::endl;
        return retCode;
    }

    // Try to connect to crazyflie
    const std::string str_uri{commandlineArguments["radiouri"]};
    Crazyflie cf(str_uri);
    cf.init();
    if (!cf.isRunning())
    {
        std::cerr << "Failed to initialize Crazyflie" << std::endl;
        return retCode;
    }
    std::cout << "Initialize Crazyflie..." << std::endl;    

    // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};    
    // Handler to receive distance readings (realized as C++ lambda).
    std::mutex Mutex;
    bitcraze::crazyflieLinkCpp::Packet packet;
    bool isCommandReceived = false;
    float cur_x{0};
    float cur_y{0};
    float cur_z{0};
    float cur_yaw{0};
    auto onCommandReceived = [&cf, &packet, &Mutex, &isCommandReceived,
                              &cur_x, &cur_y, &cur_z, &cur_yaw](cluon::data::Envelope &&env){
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
        opendlv::logic::action::CrazyFlieCommand cfcommand = cluon::extractMessage<opendlv::logic::action::CrazyFlieCommand>(std::move(env));

        // Use the command to send to crazyflie
        std::lock_guard<std::mutex> lck(Mutex);
        switch (senderStamp) {
            case 0: // Takeoff
                packet = PacketUtils::takeoffCommand(cur_z + cfcommand.height(), 0.0f, cfcommand.time()); 
                break;
            case 1: // Land
                float height = cur_z - cfcommand.height();
                if ( height <= 0.0f ){
                    height = 0.0f;
                }
                packet = PacketUtils::landCommand(height, 0.0f, cfcommand.time()); 
                break;
            case 2: // Stop
                packet = PacketUtils::stopCommand(); 
                break;
            case 3: // Goto
                packet = PacketUtils::gotoCommand(cur_x + cfcommand.x(), cur_y + cfcommand.y(), cur_z + cfcommand.z(), cur_yaw + cfcommand.yaw(), cfcommand.time()); 
                break;
        }
        isCommandReceived = true;
        std::cout << "Set to packet" << std::endl; 
    };
    // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    od4.dataTrigger(opendlv::logic::action::CrazyFlieCommand::ID(), onCommandReceived);  
    std::cout << "Subscribe to od4." << std::endl;

    // Listen to crazyflie to get log data
    int res = cf.createLogBlock({
        {"pm", "vbat"},
        {"stateEstimate", "x"}, {"stateEstimate", "y"}, {"stateEstimate", "z"},
        {"stateEstimate", "pitch"}, {"stateEstimate", "yaw"},},
        "test");
    if (res < 0)
        std::cout << "creation Error: " << res << std::endl;
    res = cf.startLogBlock(10, "test");
    if (res < 0)
        std::cout << "starting Error: " << res << std::endl;
    
    std::mutex mu;
    std::unique_lock<std::mutex> lock(mu);
    std::mutex *muPtr = &mu;
    std::condition_variable waitTillFinished;
    std::condition_variable *waitTillFinishedPtr = &waitTillFinished;
    std::atomic<bool> isFinished(false);
    std::atomic<bool> *isFinishedPtr = &isFinished;
    std::atomic<bool> isCallbackFinished(false);
    std::atomic<bool> *isCallbackFinishedPtr = &isCallbackFinished;
    std::cout << "pass " << res << std::endl;
    cf.addLogCallback([&od4, isFinishedPtr, muPtr, waitTillFinishedPtr, isCallbackFinishedPtr
                      ,&cur_x, &cur_y, &cur_z, &cur_yaw](const std::map<TocItem,boost::spirit::hold_any>& tocItemsAndValues, uint32_t period)
    {
        std::cout <<"  period:  " << period << "  val=  ";
        for(auto element : tocItemsAndValues){            
            std::cout << element.second<<"  ";
        }
        std::cout << std::endl;
        opendlv::sim::Frame frame;
        auto it = tocItemsAndValues.begin();
        std::advance(it, 1);
        cur_x = it->second.cast<float>();
        frame.x(cur_x);
        // frame.x(1.05f);
        std::advance(it, 1);
        cur_y = it->second.cast<float>();
        frame.y(cur_y);
        // frame.y(-0.05f);
        std::advance(it, 1);
        cur_z = it->second.cast<float>();
        frame.z(cur_z);
        // frame.z(0.1f);
        // std::advance(it, 1);
        // frame.roll(it->second.cast<float>());
        std::advance(it, 1);
        frame.pitch(it->second.cast<float>() / 180.0f * M_PI);
        std::advance(it, 1);
        cur_yaw = it->second.cast<float>() / 180.0f * M_PI;
        frame.yaw(cur_yaw);
        // frame.yaw(yaw_test);   
        // yaw_test = yaw_test + 0.1;

        // frame.yaw(-5.5);  
        cluon::data::TimeStamp sampleTime;
        od4.send(frame, sampleTime, 0);

        // frame.x(0.3f);
        // frame.y(0.3f);
        // frame.z(0.1f);
        // frame.yaw(0.0f);  
        // od4.send(frame, sampleTime, 2);

        if ((bool)*isFinishedPtr)
        {
            *isCallbackFinishedPtr = true;
            waitTillFinishedPtr->notify_all();
            return false;
        }
        return true;
    },"test");

    // res = cf.createLogBlock({
    //     {"pm", "vbat"}},
    //     "test_pm");
    // if (res < 0)
    //     std::cout << "creation Error: " << res << std::endl;

    // std::mutex mu_1;
    // std::unique_lock<std::mutex> lock_1(mu_1);
    // std::mutex *muPtr_1 = &mu_1;
    // std::condition_variable waitTillFinished_1;
    // std::condition_variable *waitTillFinishedPtr_1 = &waitTillFinished_1;
    // std::atomic<bool> isFinished_1(false);
    // std::atomic<bool> *isFinishedPtr_1 = &isFinished_1;
    // std::atomic<bool> isCallbackFinished_1(false);
    // std::atomic<bool> *isCallbackFinishedPtr_1 = &isCallbackFinished_1;
    // cf.addLogCallback([isFinishedPtr_1, muPtr_1, waitTillFinishedPtr_1, isCallbackFinishedPtr_1](const std::map<TocItem,boost::spirit::hold_any>& tocItemsAndValues, uint32_t period)
    // {
    //     std::cout <<"  period:  " << period << "  battery val=  ";
    //     for(auto element : tocItemsAndValues){            
    //         std::cout << element.second<<"  ";
    //     }
    //     std::cout << std::endl;

    //     if ((bool)*isFinishedPtr_1)
    //     {
    //         *isCallbackFinishedPtr_1 = true;
    //         waitTillFinishedPtr_1->notify_all();
    //         return false;
    //     }
    //     return true;
    // },"test_pm");
    // res = cf.startLogBlock(1000, "test_pm");
    // if (res < 0)
    //     std::cout << "starting Error: " << res << std::endl;
    // std::cout << "pass " << res << std::endl;
    
    // Create thread to listen to od4
    std::thread od4Thread([&od4, &cf, isFinishedPtr, &isCommandReceived, &packet]() {           
        // Start the listening loop
        while ((bool)*isFinishedPtr == false && od4.isRunning()) {
            if ( isCommandReceived ){                
                // packet.setPort((uint8_t)0x08);          // PORT 8 = HIGH_LEVEL_COMMANDER
                cf.getCon().send(packet);
                std::cout << "OD4Session thread: Sent command." << std::endl;
                isCommandReceived = false;
            }            
        }
        std::cout << "OD4Session thread exiting." << std::endl;
    });

    // Wait for some actions to stop the process
    std::cout << "Press Enter to exit..." << std::endl;
    lock.unlock();
    // lock_1.unlock();
    std::cin.getline(nullptr, 0, '\n');
    lock.lock();
    // lock.lock();
    isFinished = true;
    // isFinished_1 = true;
    
    // Wait for all threads to finish
    if (od4Thread.joinable()) {
        od4Thread.join();
    }
    waitTillFinished.wait(lock, [isCallbackFinishedPtr]()
                          { return (bool)*isCallbackFinishedPtr; });   
    // waitTillFinished_1.wait(lock_1, [isCallbackFinishedPtr_1]()
    //                     { return (bool)*isCallbackFinishedPtr_1; });  
    std::cout << "Program exiting." << std::endl;

    retCode = 0;
    return retCode;
}