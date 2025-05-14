#include "datamgmt/datamgmt.hpp"


namespace processObserver {

/**
 * @brief Observes and processes system processes, collecting CPU utilization data.
 *
 * This function monitors system processes by periodically checking their CPU utilization. 
 * It updates a vector of processes with the latest CPU usage statistics. If a process's 
 * CPU utilization falls below a threshold, it is removed from the list. The function also 
 * listens for incoming responses from a communication pipe and adds new processes to 
 * the vector when a valid response is received. The CPU utilization data is sent to an 
 * InfluxDB server. If no response is received, the function waits for the next cycle.
 *
 * @param pipes A map containing pipes for communication with other modules, specifically used 
 *              for reading responses from the relation management module.
 * @param running An atomic boolean that indicates whether the observer process is still running.
 *
 * @return void This function does not return any value.
 */
void processObserver(std::map<Module_t, Pipe> pipes, std::atomic<bool> &running);

}