#pragma once

#include <iostream>
#include <string>
#include <curl/curl.h>
#include <fmt/core.h>

// store response
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

struct Request {
    std::string url;
    std::string username;
    std::string password;
    std::string query_request;
    std::string query_response;
};

struct Node {
    std::string name;
    const char *nameSpace;
    u_int64_t   handle;
    u_int32_t   pid;
};

struct Topic {
	std::string name;
    u_int64_t   pubsub_handle;
	u_int64_t   node_handle;
};

struct ServiceClient {
    std::string name;
    u_int64_t node_handle;
};

std::string getPayloadCreateNode(Node node) {
    return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "CREATE (:Node{{name:$name, namespace:$namespace, handle:$handle, pid:$pid, state:1}}) ",
                       "parameters": {{
                        "name": "{}",
                        "namespace": "{}",
                        "handle": "{}",
                        "pid": "{}"
                        }}
                    }}
                ]
        }}
    )", node.name, node.nameSpace, node.handle, node.pid);
}

std::string getPayloadCreatePubTopic(Topic topic, std::string type) {
    return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Node {{handle:$node_handle}}) MERGE (t:Topic{{name:$name}}) CREATE (n)-[:publishes_to]->(t) ",
                       "parameters": {{
                        "name": "{}",
                        "handle": "{}",
                        "node_handle": "{}",
                        "type": "{}"
                        }}
                    }}
                ]
        }}
    )", topic.name, topic.pubsub_handle, topic.node_handle, type);
}

std::string getPayloadCreateSubTopic(Topic topic, std::string type) {
    return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Node {{handle:$node_handle}}) MERGE (t:Topic{{name:$name}}) CREATE (n)<-[:sends_to]-(t) ",
                       "parameters": {{
                        "name": "{}",
                        "handle": "{}",
                        "node_handle": "{}",
                        "type": "{}"
                        }}
                    }}
                ]
        }}
    )", topic.name, topic.pubsub_handle, topic.node_handle, type);
}

std::string getPayloadAddServiceToNode(ServiceClient service) {
    return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Node {{handle: $node_handle}}) SET n.Services = COALESCE(n.Services, []) + $name WITH n MATCH (c:Node) WHERE $name IN c.Clients CREATE (c)-[:requests]->(n) CREATE (n)-[:response]->(c) ",
                       "parameters": {{
                        "name": "{}",
                        "node_handle": "{}"
                        }}
                    }}
                ]
        }}
    )", service.name, service.node_handle);
}

std::string getPayloadAddClientToNode(ServiceClient client) {
    return fmt::format(R"(
        {{
            "statements":
                [
                    {{ "statement": "MATCH (n:Node {{handle: $node_handle}}) SET n.Clients = COALESCE(n.Clients, []) + $name WITH n MATCH (s:Node) WHERE $name IN s.Services CREATE (n)-[:requests]->(s) CREATE (s)-[:response]->(n) ",
                       "parameters": {{
                        "name": "{}",
                        "node_handle": "{}"
                        }}
                    }}
                ]
        }}
    )", client.name, client.node_handle);
}

CURL *getCurl(Request *request) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Fehler beim Initialisieren von libcurl." << std::endl;
        return NULL;
    }

    const std::string auth = fmt::format("{}:{}", request->username, request->password);

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    curl_easy_setopt(curl, CURLOPT_URL, request->url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request->query_request.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    curl_easy_setopt(curl, CURLOPT_USERPWD, auth.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &(request->query_response));

    return curl;
}

int bringNodeToGraph(Node node) {
    struct Request *request = new Request;

    request->username = "neo4j";
    request->password = "123456789";
    request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    request->query_request = getPayloadCreateNode(node);
    CURL *curl = getCurl(request);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // std::cout << "Antwort von Neo4j:" << std::endl;
        // std::cout << request->query_response << std::endl;
    }

    return int(res);
}

int bringPubTopicToGraph(Topic topic) {
    struct Request *request = new Request;

    request->username = "neo4j";
    request->password = "123456789";
    request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    request->query_request = getPayloadCreatePubTopic(topic, "publisher");
    CURL *curl = getCurl(request);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // std::cout << "Antwort von Neo4j:" << std::endl;
        // std::cout << request->query_response << std::endl;
    }

    return int(res);
}

int bringSubTopicToGraph(Topic topic) {
    struct Request *request = new Request;

    request->username = "neo4j";
    request->password = "123456789";
    request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    request->query_request = getPayloadCreateSubTopic(topic, "subscription");
    CURL *curl = getCurl(request);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // std::cout << "Antwort von Neo4j:" << std::endl;
        // std::cout << request->query_response << std::endl;
    }

    return int(res);
}

int bringServiceToGraph(ServiceClient service) {
    struct Request *request = new Request;

    request->username = "neo4j";
    request->password = "123456789";
    request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    request->query_request = getPayloadAddServiceToNode(service);
    CURL *curl = getCurl(request);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // std::cout << "Antwort von Neo4j:" << std::endl;
        // std::cout << request->query_response << std::endl;
    }

    return int(res);
}

int bringClientToGraph(ServiceClient client) {
    struct Request *request = new Request;

    request->username = "neo4j";
    request->password = "123456789";
    request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

    request->query_request = getPayloadAddClientToNode(client);
    CURL *curl = getCurl(request);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
    } else {
        // std::cout << "Antwort von Neo4j:" << std::endl;
        // std::cout << request->query_response << std::endl;
    }

    return int(res);
}