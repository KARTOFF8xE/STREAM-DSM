#include <atomic>
#include <ipc/ipc-server.hpp>

#include "datamgmt/common.hpp"
#include "datamgmt/datamgmt.hpp"


/**
 * @brief Retrieves and sends a one-time node response to a client.
 *
 * This function fetches node-related information from an external source using a primary key.
 * It parses the response, extracts relevant node data, and determines the node's relationships,
 * including publishers, subscribers, client-server relationships.
 * The collected information is then sent to the client via the IPC server.
 *
 * @param server Reference to the IPC server used for sending messages.
 * @param client The client requesting the node response.
 * @param primaryKey The unique identifier of the node to retrieve information for.
 */
void singleTimeNodeResponse(IpcServer &server, RequestingClient &client, std::string primaryKey);

/**
 * @brief Retrieves and sends a one-time topic response to a client.
 *
 * This function fetches topic-related information from an external source using a primary key.
 * It parses the response, extracts relevant topic data, and determines the topic's relationships,
 * including publishers and subscribers.
 * The collected information is then sent to the client via the IPC server.
 *
 * @param server Reference to the IPC server used for sending messages.
 * @param client The client requesting the topic response.
 * @param primaryKey The unique identifier of the topic to retrieve information for.
 */
void singleTimeTopicResponse(IpcServer &server, RequestingClient &client, std::string primaryKey);

/**
 * @brief Monitors nodes and topics, handling client updates.
 *
 * This function continuously listens for client connections, processes incoming updates.
 * It receives various update messages, such as node, subscriber and publisher and client-server
 * relationships updates. The function runs until no clients remain connected.
 * If a Node is added the relation-Mgmt is called to bring it into the trees.
 *
 * @param server Reference to the IPC server used for sending updates.
 * @param pipes File descriptor for reading client data.
 * @param running Atomic flag indicating whether the observer should continue running.
 */
void nodeAndTopicObserver(const IpcServer &server, std::map<Module_t, pipe_ns::Pipe> pipes, std::atomic<bool> &running);

bool handleSearchRequests(const IpcServer &server);

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
 * 
 * @returns If a message had been received.
 */
bool receiveNodeIsClientOfUpdate(IpcClient &ipcClient, std::vector<RequestingClient> &clients, const IpcServer &server);

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
 * 
 * @returns If a message had been received.
 */
bool receiveNodeIsServerForUpdate(IpcClient &ipcClient, std::vector<RequestingClient> &clients, const IpcServer &server);

/**
 * @brief Receives and processes a NodeIsActionClientOfUpdate message.
 *
 * This function listens for an update message indicating that a node is an action client of another node.
 * If a valid update is received, it sends the update to the relevant clients.
 * Additionally, it constructs and sends a corresponding NodeIsActionServerForUpdate message
 * to notify the action server node of the relationship.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 * 
 * @returns If a message had been received.
 */
bool receiveNodeIsActionClientOfUpdate(IpcClient &ipcClient, std::vector<RequestingClient> &clients, const IpcServer &server);

/**
 * @brief Receives and processes a NodeIsActionServerForUpdate message.
 *
 * This function listens for an update message indicating that a node is an action server for another node.
 * If a valid update is received, it sends the update to the relevant clients.
 * Additionally, it constructs and sends a corresponding NodeIsActionClientOfUpdate message
 * to notify the action client node of the relationship.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 * 
 * @returns If a message had been received.
 */
bool receiveNodeIsActionServerForUpdate(IpcClient &ipcClient, std::vector<RequestingClient> &clients, const IpcServer &server);

/**
 * @brief Receives and processes a NodeSubscribersToUpdate message.
 *
 * This function listens for an update message indicating that a node has new subscribers.
 * If a valid update is received, it sends the update to the relevant clients through the server.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 * 
 * @returns If a message had been received.
 */
bool receiveSubscribersToUpdate(IpcClient &ipcClient, std::vector<RequestingClient> &clients, const IpcServer &server);

/**
 * @brief Receives and processes a NodePublishersToUpdate message.
 *
 * This function listens for an update message indicating that a node has new publishers.
 * If a valid update is received, it sends the update to the relevant clients through the server.
 *
 * @param ipcClient Reference to the IPC client used for receiving messages.
 * @param clients Vector of Client objects representing the connected clients.
 * @param server Reference to the IPC server used for sending updates.
 * 
 * @returns If a message had been received.
 */
bool receivePublishersToUpdate(IpcClient &ipcClient, std::vector<RequestingClient> &clients, const IpcServer &server);

/**
 * @brief Handles client requests by adding or removing the client from the list of clients.
 *
 * This function processes client updates by either adding or removing a client from the
 * vector of clients. It checks if the client exists in the list based on the `pid` and 
 * `requestId`. If the client has updates, the client is added to the list if it doesn't 
 * already exist. Otherwise, the client is removed from the list.
 *
 * @param client A reference to the client whose details are processed.
 * @param clients A reference to a vector of clients that will be modified (client added or removed).
 *
 * @return void This function does not return any value.
 */
void handleClient(RequestingClient &client, std::vector<RequestingClient> &clients);
