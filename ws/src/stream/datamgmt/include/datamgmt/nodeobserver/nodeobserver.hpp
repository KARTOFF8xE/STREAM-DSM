#include <atomic>
#include <ipc/ipc-server.hpp>

#include "datamgmt/common.hpp"


void singleTimeNodeResponse(IpcServer &server, Client client, primaryKey_t primaryKey);

void nodeObserver(const IpcServer &server, int pipe_r, std::atomic<bool> &running);

/**
 * @brief Receives and processes a NodeIsClientOfUpdate message.
 *
 * This function listens for an update message indicating that a node is a client of another node.
 * If a valid update is received, it processes the information and sends appropriate updates
 * to the relevant clients through the server.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 */
void receiveNodeIsClientOfUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server);

/**
 * @brief Receives and processes a NodeIsServerForUpdate message.
 *
 * This function listens for an update message indicating that a node is a server for another node.
 * If a valid update is received, it processes the information and sends appropriate updates
 * to the relevant clients through the server.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 */
void receiveNodeIsServerForUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server);

/**
 * @brief Receives and processes a NodeSubscribersToUpdate message.
 *
 * This function listens for an update message indicating that a node has new subscribers.
 * If a valid update is received, it sends the update to the relevant clients through the server.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 */
void receiveNodeSubscribersToUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server);

/**
 * @brief Receives and processes a NodePublishersToUpdate message.
 *
 * This function listens for an update message indicating that a node has new publishers.
 * If a valid update is received, it sends the update to the relevant clients through the server.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 */
void receiveNodePublishersToUpdate(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server);


void receiveNodeResponse(IpcClient &ipcClient, std::vector<Client> &clients, const IpcServer &server);

void handleClient(int ret, Client &client, std::vector<Client> &clients);
