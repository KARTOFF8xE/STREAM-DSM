#include "datamgmt/datamgmt.hpp"


namespace relationMgmt {

/**
 * @brief Manages the creation and linking of namespaces and processes based on incoming data.
 *
 * This function listens for node responses from the `NODEANDTOPICOBSERVER` module and 
 * processes the received data. It creates namespaces and links passive helpers using 
 * GraphDB. If the node response indicates a new process, the function recursively collects 
 * process data by traversing the process tree and sends the process information to the 
 * `PROCESSOBSERVER` module. The function ensures proper synchronization and manages 
 * communication between modules via pipes.
 *
 * @param pipes A map containing communication pipes for interaction with other modules, 
 *              including pipes for reading node responses and writing to the process observer.
 * @param running An atomic boolean that indicates whether the relation management process 
 *                is still running.
 *
 * @return void This function does not return any value.
 */
void relationMgmt(std::map<Module_t, Pipe> pipes, std::atomic<bool> &running);

}