#include <iostream>
#include <thread>        
#include <chrono>         
#include <vector>

#include "messages.hpp" // derived from custom odvd

#include "cluon-complete.hpp" // need this for odvd messages

#include "ConnectionWorker.h"
#include "Crazyflie.h"
#include "crazyflieLinkCpp/Packet.hpp"
#include "crazyflieLinkCpp/PacketUtils.hpp"

using bitcraze::crazyflieLinkCpp::Connection;
using bitcraze::crazyflieLinkCpp::Packet;


// This function creates and starts log blocks
int createAndStartLogBlock(Crazyflie &cf, const std::vector<std::pair<std::string, std::string>> &logItems, const std::string &blockName, int period) {
    int res = cf.createLogBlock(logItems, blockName);
    if (res < 0) {
        std::cout << "Creation Error for block " << blockName << ": " << res << std::endl;
        return res;
    }

    res = cf.startLogBlock(period, blockName);
    if (res < 0) {
        std::cout << "Starting Error for block " << blockName << ": " << res << std::endl;
    }
    return res;
}


int main(int argc, char **argv)
{

    Crazyflie cf("radio://0/90/2M/E7E7E7E7E7");
    cf.init();

    if (!cf.isRunning())
    {
        std::cerr << "Failed to initialize Crazyflie" << std::endl;
        return -1;
    }

     // create and start log blocks
    std::cout << "Taking off" << std::endl;
    Packet p_takeoff = PacketUtils::takeoffCommand(2.0f, 0.5f, 2.0f);
    cf.getCon().send(p_takeoff);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    int32_t logPeriod = 5000;

    std::vector<std::pair<std::string, std::string>> logPosOr = {
        {"stateEstimate", "x"}, {"stateEstimate", "y"}, {"stateEstimate", "z"},
        {"stabilizer", "roll"}, {"stabilizer", "pitch"}, {"stabilizer", "yaw"}
    };

    int32_t res = createAndStartLogBlock(cf, logPosOr, "logPosOr", logPeriod);
    if (res < 0) {
        std::cerr << "Error processing log block logPosOr: " << res << std::endl;
    } else {
        std::cout << "Successfully created and started logPosOr block" << std::endl;
    }

   
    // receiving data from the systems connected to 111(refer to opendlv)

    // this part needs modifications
    cluon::OD4Session od4{111, [](cluon::data::Envelope &&envelope) noexcept {
        if (envelope.dataType() == PositionData::ID()) { // PositionData ID
            PositionData receivedMsg = cluon::extractMessage<PositionData>(std::move(envelope));
            std::cout << "Received PositionData - x: " << receivedMsg.x() << " y: " << receivedMsg.y() << " z: " << receivedMsg.z() << std::endl;
        } else if (envelope.dataType() == OrientationData::ID()) { // OrientationData ID
            OrientationData receivedMsg = cluon::extractMessage<OrientationData>(std::move(envelope));
            std::cout << "Received OrientationData - roll: " << receivedMsg.roll() << " pitch: " << receivedMsg.pitch() << " yaw: " << receivedMsg.yaw() << std::endl;
        } else {
            std::cout<<" Unknown"<<std::endl;
        }}
    };

    // printing the data and sending them to systems connected over 111(refer to opendlv)
    std::mutex mu;
    std::unique_lock<std::mutex> lock(mu);
    std::mutex *muPtr = &mu;
    std::condition_variable waitTillFinished;
    std::condition_variable *waitTillFinishedPtr = &waitTillFinished;
    std::atomic<bool> isFinished(false);
    std::atomic<bool> *isFinishedPtr = &isFinished;
    std::atomic<bool> isCallbackFinished(false);
    std::atomic<bool> *isCallbackFinishedPtr = &isCallbackFinished;

    cf.addLogCallback([isFinishedPtr, muPtr, waitTillFinishedPtr, isCallbackFinishedPtr, &od4]
    (const std::map<TocItem, boost::spirit::hold_any>& tocItemsAndValues, uint32_t period) {
        std::lock_guard<std::mutex> lock(*muPtr);

        for (const auto& element : tocItemsAndValues) {
            try {
                if (element.first._groupName == "stateEstimate" && (element.first._name == "x" || element.first._name == "y" || element.first._name == "z")) {
                    PositionData posData;
                    if (element.first._name == "x") {
                        posData.x(boost::spirit::any_cast<float>(element.second));
                        std::cout << "x: " << posData.x() << " ";
                    } else if (element.first._name == "y") {
                        posData.y(boost::spirit::any_cast<float>(element.second));
                        std::cout << "y: " << posData.y() << " ";
                    } else if (element.first._name == "z") {
                        posData.z(boost::spirit::any_cast<float>(element.second));
                        std::cout << "z: " << posData.z() << " ";
                    }
                    od4.send(posData);
                } else if (element.first._groupName == "stabilizer" && (element.first._name == "roll" || element.first._name == "pitch" || element.first._name == "yaw")) {
                    OrientationData orData;
                    if (element.first._name == "roll") {
                        orData.roll(boost::spirit::any_cast<float>(element.second));
                        std::cout << "roll: " << orData.roll() << " ";
                    } else if (element.first._name == "pitch") {
                        orData.pitch(boost::spirit::any_cast<float>(element.second));
                        std::cout << "pitch: " << orData.pitch() << " ";
                    } else if (element.first._name == "yaw") {
                        orData.yaw(boost::spirit::any_cast<float>(element.second));
                        std::cout << "yaw: " << orData.yaw() << " ";
                    }
                    od4.send(orData);
                } else {
                    std::cout << "Unknown: " << element.second << " ";
                }
            } catch (const std::exception& e) {
                std::cerr << "Error during callback: " << e.what() << std::endl;
            }
        }
        std::cout << std::endl;

        if ((bool)*isFinishedPtr) {
            *isCallbackFinishedPtr = true;
            waitTillFinishedPtr->notify_all();
            return false;
        }
        return true;
    }, "logPosOr");

    std::cout << "Press enter to stop receiving" << std::endl;
    lock.unlock();
    std::cin.getline(nullptr, 0, '\n');
    lock.lock();
    isFinished = true;
    waitTillFinished.wait(lock, [isCallbackFinishedPtr]() { return (bool)*isCallbackFinishedPtr; });

    std::cout << "Landing..." << std::endl;
    cf.getCon().send(PacketUtils::landCommand(0.0f, 0.0f, 2.0f));
    std::this_thread::sleep_for(std::chrono::milliseconds(2250));

    std::cout << "Stopping..." << std::endl;
    cf.getCon().send(PacketUtils::stopCommand());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    cf.getCon().close();
    std::cout << "Done." << std::endl;

    return 0;

}
