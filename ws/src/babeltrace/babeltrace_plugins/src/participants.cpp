#pragma once

#include <iostream>
#include <string>
#include <curl/curl.h>
#include <fmt/core.h>

#include <babeltrace2/babeltrace.h>

#include "interface.cpp"

struct Request {
    std::string url;
    std::string username;
    std::string password;
    std::string query_request;
    std::string query_response;
};

// store response
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
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

class FooNode: public Participant {
    private:
        std::string name;
        const char *nameSpace;
        u_int64_t   handle;
        u_int32_t   pid;

        std::string getPayload() override {
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
            )", this->name, this->nameSpace, this->handle, this->pid);
        }

    public:
        void extractInfo(const bt_event *event) override {
            const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
            const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                    const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_name");
                    this->name = std::string(bt_field_string_get_value(field));
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "namespace");
                    this->nameSpace = bt_field_string_get_value(field);
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
                    this->handle = bt_field_integer_unsigned_get_value(field);
            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

            const bt_field *ctx_field = bt_event_borrow_common_context_field_const(event);
            field_class = bt_field_borrow_class_const(ctx_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(ctx_field, "vpid");
                this->pid = uint32_t(bt_field_integer_signed_get_value(field));
            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }

            return;
        }

        void toGraph() override {
            struct Request *request = new Request;

            request->username = "neo4j";
            request->password = "123456789";
            request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

            request->query_request = getPayload();
            CURL *curl = getCurl(request);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
            } else {
                // std::cout << "Antwort von Neo4j:" << std::endl;
                // std::cout << request->query_response << std::endl;
            }
        }

};

class FooPublisher: public Participant {
    private:
        std::string name;
        u_int64_t   pubsub_handle;
        u_int64_t   node_handle;

        std::string getPayload() override {
            return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Node {{handle:$node_handle}}) MERGE (t:Topic{{name:$name}}) CREATE (n)-[:publishes_to]->(t) ",
                        "parameters": {{
                            "name": "{}",
                            "handle": "{}",
                            "node_handle": "{}"
                            }}
                        }}
                    ]
            }}
        )", this->name, this->pubsub_handle, this->node_handle);
        }

    public:
        void extractInfo(const bt_event *event) override {
            const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
            const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                    const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "topic_name");
                    this->name = std::string(bt_field_string_get_value(field));
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
                    this->node_handle = bt_field_integer_unsigned_get_value(field);
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "publisher_handle");
                    this->pubsub_handle = bt_field_integer_unsigned_get_value(field);
            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
        }

        void toGraph() override {
            struct Request *request = new Request;

            request->username = "neo4j";
            request->password = "123456789";
            request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

            request->query_request = getPayload();
            CURL *curl = getCurl(request);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
            } else {
                // std::cout << "Antwort von Neo4j:" << std::endl;
                // std::cout << request->query_response << std::endl;
            }
        }
};

class FooSubscriber: public Participant {
    private:
        std::string name;
        u_int64_t   pubsub_handle;
        u_int64_t   node_handle;

        std::string getPayload() override{
            return fmt::format(R"(
            {{
                "statements":
                    [
                        {{ "statement": "MATCH (n:Node {{handle:$node_handle}}) MERGE (t:Topic{{name:$name}}) CREATE (n)<-[:sends_to]-(t) ",
                        "parameters": {{
                            "name": "{}",
                            "handle": "{}",
                            "node_handle": "{}"
                            }}
                        }}
                    ]
            }}
        )", this->name, this->pubsub_handle, this->node_handle);
        }

    public:
        void extractInfo(const bt_event *event) override {
            const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
            const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                    const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "topic_name");
                    this->name = std::string(bt_field_string_get_value(field));
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
                    this->node_handle = bt_field_integer_unsigned_get_value(field);
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "subscription_handle");
                    this->pubsub_handle = bt_field_integer_unsigned_get_value(field);
            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
        }

        void toGraph() override {
            struct Request *request = new Request;

            request->username = "neo4j";
            request->password = "123456789";
            request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

            request->query_request = getPayload();
            CURL *curl = getCurl(request);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
            } else {
                // std::cout << "Antwort von Neo4j:" << std::endl;
                // std::cout << request->query_response << std::endl;
            }
        }
};

class FooService: public Participant {
    private:
        std::string name;
        u_int64_t node_handle;

        std::string getPayload() override{
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
        )", this->name, this->node_handle);
        }

    public:
        void extractInfo(const bt_event *event) override {
            const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
            const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                    const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "service_name");
                    this->name = std::string(bt_field_string_get_value(field));
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
                    this->node_handle = bt_field_integer_unsigned_get_value(field);

            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
        }

        void toGraph() override {
            struct Request *request = new Request;

            request->username = "neo4j";
            request->password = "123456789";
            request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

            request->query_request = getPayload();
            CURL *curl = getCurl(request);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
            } else {
                // std::cout << "Antwort von Neo4j:" << std::endl;
                // std::cout << request->query_response << std::endl;
            }
        }
};

class FooClient: public Participant {
    private:
        std::string name;
        u_int64_t node_handle;

        std::string getPayload() override{
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
        )", this->name, this->node_handle);
        }

    public:
        void extractInfo(const bt_event *event) override {
            const bt_field *payload_field = bt_event_borrow_payload_field_const(event);
            const bt_field_class *field_class = bt_field_borrow_class_const(payload_field);
            if (bt_field_class_get_type(field_class) == BT_FIELD_CLASS_TYPE_STRUCTURE) {
                    const bt_field *field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "service_name");
                    this->name = std::string(bt_field_string_get_value(field));
                    field = bt_field_structure_borrow_member_field_by_name_const(payload_field, "node_handle");
                    this->node_handle = bt_field_integer_unsigned_get_value(field);

            } else { printf("\033[33;1WRONG TYPE\033[0m\n"); }
        }

        void toGraph() override {
            struct Request *request = new Request;

            request->username = "neo4j";
            request->password = "123456789";
            request->url = "http://172.17.0.1:7474/db/neo4j/tx/commit";

            request->query_request = getPayload();
            CURL *curl = getCurl(request);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "Fehler bei der Anfrage: " << curl_easy_strerror(res) << std::endl;
            } else {
                // std::cout << "Antwort von Neo4j:" << std::endl;
                // std::cout << request->query_response << std::endl;
            }
        }
};

class FooUnknown: public Participant {
    private:
        std::string getPayload() override {
            return "";
        }

    public:
        void extractInfo(const bt_event *) override {
            std::cout << "Found unknown Type, not handled" << std::endl;
        }

        void toGraph() override{}
};

class ParticipantFactory {
    public:
        static Participant *getParticipant(const char *event_name) {
            if (strcmp(
                "ros2:rcl_node_init",
                event_name) == 0) {
                    FooNode *fooNode = new FooNode();
                    return fooNode;
            }
            if (strcmp(
                "ros2:rcl_publisher_init",
                event_name) == 0) {
                    FooPublisher *fooPublisher = new FooPublisher();
                    return fooPublisher;
            }
            if (strcmp(
                "ros2:rcl_subscription_init",
                event_name) == 0) {
                    FooSubscriber *fooSubscriber = new FooSubscriber();
                    return fooSubscriber;
            }
            if (strcmp(
                "ros2:rcl_service_init",
                event_name) == 0) {
                    FooService *fooService = new FooService();
                    return fooService;
            }
            if (strcmp(
                "ros2:rcl_client_init",
                event_name) == 0) {
                    FooClient *fooClient = new FooClient();
                    return fooClient;
            }
            
            return new FooUnknown();
        }
};