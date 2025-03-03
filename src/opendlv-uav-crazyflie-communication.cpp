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

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>
#include <chrono>

#include <cmath>

struct log {
  float x;
  float y;
  float z;
  float pitch;
  float yaw;
  float pm_vbat;
} __attribute__((packed));

struct command {
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float yawRate;
  float height;
  float time;
  int16_t Type;
} __attribute__((packed));

volatile bool g_done = false;

void onLogData(uint32_t /*time_in_ms*/, const struct log* data)
{
    // std::cout << data->pm_extVbat << std::endl;
    // std::cout << data->pm_extCurr << std::endl;
    g_done = true;
}

bool InitializeCrazyflie(std::unique_ptr<Crazyflie>& cf, std::unique_ptr<LogBlock<struct log> >& logBlock, const std::string& uri, cluon::OD4Session& od4, bool verbose, bool test_mode, int16_t frame_id) {
    std::cout << "Initializing Crazyflie..." << std::endl;
    try{
        cf.reset(new Crazyflie(uri));
        cf->logReset();
        cf->requestLogToc();

        // std::unique_ptr<LogBlock<struct log> > logBlock;
        std::function<void(uint32_t, const struct log*)> cb = 
        [&od4, &verbose, &test_mode, &frame_id](uint32_t /*time_in_ms*/, const struct log* data) {
            if ( verbose ){
                std::cout << "Message received, x:" << data->x << ", y:" << data->y << ", z:" << data->z << ", pitch:" << data->pitch << ", yaw:" << data->yaw << ", voltage:" << data->pm_vbat << std::endl;
            }

            // Send message by od4
            opendlv::sim::Frame frame;
            opendlv::logic::sensation::CrazyFlieState cfState;
            cfState.battery_state(data->pm_vbat);
            cfState.cur_yaw(data->yaw / 180.0f * M_PI);

            frame.x(data->x);
            frame.y(data->y);
            frame.z(data->z);
            frame.pitch(data->pitch / 180.0f * M_PI);
            frame.yaw(data->yaw / 180.0f * M_PI);
            
            // if ( test_mode ){
            //     frame.x(1.0f);
            //     frame.y(0.0f);
            //     frame.z(1.0f);
            //     frame.yaw(180.0f / 180.0f * M_PI);
            // }

            cluon::data::TimeStamp sampleTime;
            od4.send(frame, sampleTime, frame_id);
            od4.send(cfState, sampleTime, frame_id);

            g_done = true;
        };

        logBlock.reset(new LogBlock<struct log>(
            cf.get(),{
            {"stateEstimate", "x"},
            {"stateEstimate", "y"},
            {"stateEstimate", "z"},
            {"stateEstimate", "pitch"},
            {"stateEstimate", "yaw"},
            {"pm", "vbat"}
            // {"pm", "chargeCurrent"}
            }, cb));
        logBlock->start(1); // 100ms -> 10

        // Check that whether the connection succeed
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // while (true) {
        //     cf->sendPing();
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
        // std::cout << "Connection succeed!!" << std::endl;

        return true;
    }
    catch(std::exception& e){
        std::cerr << "Initialize failed due to: " << e.what() << std::endl;
        return false;
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
    
    if ( (0 == commandlineArguments.count("frameId")) ) {
        std::cerr << "You should include the frameId to specify which crazyflie are you refering to" << std::endl;
        return retCode;
    }

    const std::string str_uri{commandlineArguments["radiouri"]};
    const int16_t frame_id{ static_cast<int16_t>(std::stoi(commandlineArguments["frameId"])) };
    bool const verbose{commandlineArguments.count("verbose") != 0};
    bool const test_mode{commandlineArguments.count("test_mode") != 0};

    // Create a od4 session
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    // Try to connect to crazyflie
    std::unique_ptr<Crazyflie> cf;
    std::unique_ptr<LogBlock<struct log> > logBlock;
    if ( !InitializeCrazyflie( cf, logBlock, str_uri, od4, verbose, test_mode, frame_id) )
        return 1;    
    std::cout << "Connected to crazyflie." << std::endl;

    // Connect to the od4 session 
    std::mutex Mutex;
    command inputCommand;
    bool isCommandReceived = false;
    auto onCommandReceived = [&Mutex, &inputCommand, &isCommandReceived](cluon::data::Envelope &&env){
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
        opendlv::logic::action::CrazyFlieCommand cfcommand = cluon::extractMessage<opendlv::logic::action::CrazyFlieCommand>(std::move(env));

        // Use the command to send to crazyflie
        std::lock_guard<std::mutex> lck(Mutex);
        switch (senderStamp) {
            case 0: // Takeoff
                inputCommand.Type = 0;
                inputCommand.height = cfcommand.height();
                inputCommand.time = cfcommand.time();
                break;
            case 1: // Land
                inputCommand.Type = 1;
                inputCommand.height = cfcommand.height();
                inputCommand.time = cfcommand.time();
                break;
            case 2: // Stop
                inputCommand.Type = 2;
                break;
            case 3: // Goto
                inputCommand.Type = 3;
                inputCommand.x = cfcommand.x();
                inputCommand.y = cfcommand.y();
                inputCommand.z = cfcommand.z();
                inputCommand.yaw = cfcommand.yaw();
                inputCommand.time = cfcommand.time();
                break;
            case 4: // Hovering
                inputCommand.Type = 4;
                inputCommand.vx = cfcommand.vx();
                inputCommand.vy = cfcommand.vy();
                inputCommand.yawRate = cfcommand.yawRate();
                inputCommand.z = cfcommand.z();
                break;
        }
        isCommandReceived = true;
        std::cout << "Command received with type: " << senderStamp << std::endl; 
    };
    // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
    od4.dataTrigger(opendlv::logic::action::CrazyFlieCommand::ID(), onCommandReceived);  
    std::cout << "Subscribe to od4." << std::endl;

    // Start the looping here
    while(od4.isRunning()){
        // std::cout << "Loop start..." << std::endl;
        try{
            if (true) {
                cf->sendPing();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            // continue;
            // std::cout << "Connection succeed!!" << std::endl;

            // std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if ( isCommandReceived == false )
                continue;

            int16_t group_mask = 0;
            std::cout << "Received command..." << std::endl;
            switch (inputCommand.Type)
            {
                case 0: // Takeoff
                    cf->takeoff(inputCommand.height, inputCommand.time, group_mask);
                    break;
                case 1: // Land
                    cf->land(inputCommand.height, inputCommand.time, group_mask);
                    break;
                case 2: // Stop
                    cf->stop(group_mask);
                    break;
                case 3: // Goto
                    {
                        bool relative = true;
                        cf->goTo(inputCommand.x, inputCommand.y, inputCommand.z, inputCommand.yaw, inputCommand.time, relative, group_mask);
                        break;                        
                    }                    
                case 4: // Hovering
                    cf->sendHoverSetpoint(inputCommand.vx, inputCommand.vy, inputCommand.yawRate, inputCommand.z);
                    break;
            }
            isCommandReceived = false;
        }
        catch(std::exception& e){
            std::cerr << "Has some error with: " << e.what() << std::endl;
            if ( !InitializeCrazyflie( cf, logBlock, str_uri, od4, verbose, test_mode, frame_id) )
                return 1;    
            std::cout << "Reconnected to crazyflie, sleep for a while..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }        
    }

    retCode = 0;
    return retCode;
}