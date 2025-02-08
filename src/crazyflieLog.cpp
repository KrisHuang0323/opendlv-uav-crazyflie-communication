#include "Crazyflie.h"

int main()
{
    Crazyflie cf("radio://0/90/2M/E7E7E7E7E7");
    cf.init();
    int res = cf.createLogBlock({
        {"stateEstimate", "x"}, {"stateEstimate", "y"}, {"stateEstimate", "z"},
        {"stateEstimate", "roll"}, {"stateEstimate", "pitch"}, {"stateEstimate", "yaw"},
                                },
                                "test");

    if (res < 0)
        std::cout << "creation Error: " << res << std::endl;
    res = cf.startLogBlock(10, "test");
    if (res < 0)
        std::cout << "starting Error: " << res << std::endl;
    // res = cf.createLogBlock({
    //                                 {"stabilizer", "yaw"},
    //                                 {"stabilizer","roll"},
    //                             },
    //                             "test2");
    // if (res < 0)
    //     std::cout << "creation Error: " << res << std::endl;
    // res = cf.startLogBlock(10, "test2");
    // if (res < 0)
    //     std::cout << "starting Error: " << res << std::endl;

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
    cf.addLogCallback([ isFinishedPtr, muPtr, waitTillFinishedPtr, isCallbackFinishedPtr]
    (const std::map<TocItem,boost::spirit::hold_any>& tocItemsAndValues, uint32_t period)
                      {
                        std::cout <<"  period:  " << period << "  val=  ";
                        for(auto element : tocItemsAndValues)
                        {
                        
                            // if(to_string(element.first._type).find("uint")!=std::string::npos)
                            // {
                            //     uint32_t res = 0;
                            //     std::memcpy(&res, element.second,element.first._type.size());
                            //     std::cout << res;
                            // }
                            // else if (element.first._type == "int8_t")
                            // {
                            //     std::cout << (int)*(int8_t*)element.second;
                            // }
                            //  else if (element.first._type == "int16_t")
                            // {
                            //     std::cout << *(int16_t*)element.second;
                            // }
                            //  else if(element.first._type == "int32_t")
                            // {
                            //     std::cout << *(int32_t*)element.second;
                            // }
                            // else if (element.first._type == "float")
                            // {
                            //     std::cout << *(float*)element.second;
                            // }
                            std::cout << element.second<<"  ";
                        }
                        std::cout << std::endl;

                        if ((bool)*isFinishedPtr)
                        {
                            *isCallbackFinishedPtr = true;
                            waitTillFinishedPtr->notify_all();
                            return false;
                        }
                        return true;
                      },"test");
                      
    std::cout << "Press enter to stop receiving" << std::endl;
    lock.unlock();
    std::cin.getline(nullptr, 0, '\n');
    lock.lock();
    isFinished = true;
    waitTillFinished.wait(lock, [isCallbackFinishedPtr]()
                          { return (bool)*isCallbackFinishedPtr; });
    return 0;
}